#!/usr/bin/env python3
"""Raporun uc ortamini dokulu olarak uretir (gorsel SLAM icin).

    tools/dokulu_dunya.py --tip koridor                  # uzun koridor
    tools/dokulu_dunya.py --tip oda                      # dagnik oda
    tools/dokulu_dunya.py --tip engebe --seviye 2        # hafif engebeli zemin
    tools/dokulu_dunya.py --tip koridor --mu 0.2 --ad koridor_kaygan_m02

Neden: mevcut koridor duz gri kutulardan olusuyor ve temiz goruntude bile
yalnizca ~57 ORB kosesi / 27 eslesme veriyor. ORB-SLAM3 ve RTAB-Map'in gorsel
odometrisi bu veriyle HER kosulda basarisiz olur, dolayisiyla zorlu kosullarin
etkisi olculemez. Bu betik PBR albedo dokulu bir koridor uretir.

Duvarlar 1 m'lik PANELLERE bolunuyor. Sebebi: Gazebo ilkel kutu yuzeyine
dokuyu BIR KEZ geriyor, 16 m'lik tek kutuda doku asiri dusuk uzamsal frekansa
iner ve kose uretmez. Her panele ayri doku verilince hem uzamsal frekans
dogru olur hem de paneller birbirinden ayirt edilebilir (tekrarli doku yanlis
eslesme uretir; olculdu: ayni panel 1949 eslesme, farkli paneller 20).

Doku yollari MUTLAK yazilir. gz, dunya dosyasina gore bagil yollari guvenilir
sekilde cozemiyor; dosya uretilen oldugu icin mutlak yol kabul edilebilir —
dokular degisirse betik yeniden kosturulur.
"""
import argparse, os
import numpy as np

# Engebe icin heightmap: 1500+ ayri kutuyla denendiginde gercek zaman carpani
# 0.07'ye dusuyordu, heightmap tek bir carpisma yuzeyi oldugu icin neredeyse
# bedava. (Bu bulgu tools/engebe_dunyasi.py'den geliyor, orada olculmustu.)
ENGEBE_N = 513            # 2^n + 1 olmali
ENGEBE_SEVIYELERI = {0: 0.0, 1: 0.02, 2: 0.06, 3: 0.12}   # tepe-dip araligi (m)

BASLIK = """<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="{ad}">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-particle-emitter-system" name="gz::sim::systems::ParticleEmitter"/>
    <!-- Contact sistemi OLMADAN ayak temas sensorleri hic yayin yapmiyor; o
         zaman butun ayaklar "havada" gorunuyor ve bacak odometrisi hizi
         sifirlanip DONUYOR. Sessiz bir hata: kayit normal gorunuyor, yalnizca
         /odom/raw sabit kaliyor (olculdu: 3 kayitta birden). -->
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <scene>
      <ambient>{ambient} {ambient} {ambient} 1</ambient>
      <background>0.7 0.75 0.8 1</background>
      <grid>false</grid>
    </scene>
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>{gunes} {gunes} {gunes} 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.4 0.2 -0.9</direction>
    </light>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
          <surface><friction><ode><mu>{mu}</mu><mu2>{mu}</mu2></ode></friction></surface>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
          <material><ambient>0.5 0.5 0.5 1</ambient><diffuse>0.5 0.5 0.5 1</diffuse></material>
        </visual>
      </link>
    </model>
"""

GORSEL_KUTU = """    <model name="{ad}">
      <static>true</static>
      <pose>{x} {y} {z} 0 0 {yaw}</pose>
      <link name="link">
{cars}        <visual name="visual">
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
          <material>
            <diffuse>1 1 1 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
            <pbr><metal>
              <albedo_map>{doku}</albedo_map>
              <roughness>0.85</roughness>
              <metalness>0.0</metalness>
            </metal></pbr>
          </material>
        </visual>
      </link>
    </model>
"""

CARPISMA = """        <collision name="collision">
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
        </collision>
"""


HEIGHTMAP = """    <model name="engebe">
      <static>true</static>
      <pose>{cx} 0 0 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <heightmap>
              <uri>file://{png}</uri>
              <size>{boyut} {boyut} {h:.4f}</size>
              <pos>0 0 0</pos>
            </heightmap>
          </geometry>
          <surface><friction><ode><mu>{mu}</mu><mu2>{mu}</mu2></ode></friction></surface>
        </collision>
        <visual name="visual">
          <geometry>
            <heightmap>
              <use_terrain_paging>false</use_terrain_paging>
              <uri>file://{png}</uri>
              <size>{boyut} {boyut} {h:.4f}</size>
              <pos>0 0 0</pos>
              <texture>
                <diffuse>file://{doku}</diffuse>
                <normal>file://{normal}</normal>
                <size>2</size>
              </texture>
            </heightmap>
          </geometry>
        </visual>
      </link>
    </model>
"""


def yukseklik_resmi(h_arali, tohum, baslangic_x, merkez_x, boyut):
    """Duz baslangic bolgesi + yumusak gecisli cok olcekli gurultu."""
    from PIL import Image
    rnd = np.random.default_rng(tohum)
    h = np.zeros((ENGEBE_N, ENGEBE_N))
    for olcek, agirlik in ((8, 1.0), (16, 0.5), (32, 0.25)):
        kaba = rnd.random((olcek + 1, olcek + 1))
        img = Image.fromarray((kaba * 255).astype(np.uint8)).resize(
            (ENGEBE_N, ENGEBE_N), Image.BICUBIC)
        h += agirlik * (np.asarray(img, dtype=float) / 255.0)
    h = (h - h.min()) / (h.max() - h.min() + 1e-9)
    # Robot DUZ bolgede dogup sonra engebeye girsin: nerede bozuldugu
    # dogrudan yorungeden okunur.
    j = np.arange(ENGEBE_N)
    dunya_x = merkez_x - boyut / 2 + (j / (ENGEBE_N - 1)) * boyut
    gecis = np.clip((dunya_x - baslangic_x) / 2.0, 0.0, 1.0)
    return (h * gecis[None, :] * 255).astype(np.uint8)


def nokta_dogru_uzakligi(px, py, a, b):
    """(px,py) noktasinin [a,b] dogru parcasina uzakligi."""
    ax, ay = a; bx, by = b
    dx, dy = bx - ax, by - ay
    uzun = dx * dx + dy * dy
    t = 0.0 if uzun < 1e-12 else max(0.0, min(1.0, ((px-ax)*dx + (py-ay)*dy) / uzun))
    return float(np.hypot(px - (ax + t*dx), py - (ay + t*dy)))


def kutu(ad, x, y, z, sx, sy, sz, doku, carpisma=True, yaw=0.0):
    return GORSEL_KUTU.format(
        ad=ad, x=x, y=y, z=z, sx=sx, sy=sy, sz=sz, doku=doku, yaw=yaw,
        cars=CARPISMA.format(sx=sx, sy=sy, sz=sz) if carpisma else "")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tip", choices=("koridor", "oda", "engebe"), default="koridor",
                    help="raporun uc ortamindan biri")
    ap.add_argument("--ad", default=None, help="varsayilan: tipten turetilir")
    ap.add_argument("--uzunluk", type=float, default=None, help="x boyutu (m)")
    ap.add_argument("--genislik", type=float, default=None, help="y boyutu (m)")
    ap.add_argument("--yukseklik", type=float, default=2.0)
    ap.add_argument("--panel", type=float, default=1.0, help="duvar panel uzunlugu (m)")
    ap.add_argument("--doku-sayisi", type=int, default=8,
                    help="dongude kullanilan duvar dokusu sayisi (1 = tekrarli, zayif)")
    ap.add_argument("--mu", type=float, default=0.8,
                    help="zemin surtunme katsayisi; KAYGAN ZEMIN kosulunun seviyesi")
    ap.add_argument("--engel", type=int, default=None,
                    help="dokulu kutu sayisi. koridor icin varsayilan 0 "
                         "('uzun koridor' engelsiz), oda icin 14.")
    ap.add_argument("--rota", default=None,
                    help="engellerin uzak duracagi rota, rota_surucu sozdizimi "
                         "(orn. kare:6). Verilmezse tipe gore secilir.")
    ap.add_argument("--serbest-serit", type=float, default=2.2,
                    help="rota boyunca birakilan engelsiz serit genisligi (m)")
    ap.add_argument("--seviye", type=int, default=2, choices=sorted(ENGEBE_SEVIYELERI),
                    help="engebe siddeti (yalnizca --tip engebe)")
    ap.add_argument("--baslangic-x", type=float, default=3.0,
                    help="engebenin basladigi x (oncesi duz)")
    ap.add_argument("--arka-pay", type=float, default=None,
                    help="baslangic noktasinin (0,0) arkasinda birakilan bosluk (m). "
                         "Oda icin zorunlu: arka duvar dogrudan x=0'a konunca "
                         "robot duvarin ICINDE doguyor ve hic hareket edemiyor.")
    ap.add_argument("--ambient", type=float, default=0.45)
    ap.add_argument("--gunes", type=float, default=0.85)
    ap.add_argument("--tohum", type=int, default=0)
    ap.add_argument("--cikti", default=None)
    a = ap.parse_args()

    VARSAYILAN = {               # (uzunluk, genislik, engel, rota)
        "koridor": (16.0, 4.0, 0, "duz:10"),
        "oda":     (10.0, 10.0, 14, "kare:4"),
        "engebe":  (20.0, 12.0, 0, "duz:12"),
    }
    u0, g0, e0, r0 = VARSAYILAN[a.tip]
    pay = a.arka_pay if a.arka_pay is not None else (1.5 if a.tip == "oda" else 0.0)
    L = a.uzunluk if a.uzunluk is not None else u0
    W = a.genislik if a.genislik is not None else g0
    n_engel = a.engel if a.engel is not None else e0
    rota_adi = a.rota or r0
    ad = a.ad or {"koridor": "koridor_dokulu", "oda": "oda_dagnik",
                  "engebe": f"engebe_s{a.seviye}"}[a.tip]

    kok = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    acik = os.path.join(kok, "ws/src/unitree_go2_ros2/unitree_go2_description")
    dok = os.path.join(acik, "materials/textures")
    eksik = [f"duvar_{i}.png" for i in range(a.doku_sayisi)
             if not os.path.exists(os.path.join(dok, f"duvar_{i}.png"))]
    if eksik or not os.path.exists(os.path.join(dok, "zemin.png")):
        print(f"HATA: doku yok ({eksik or 'zemin.png'}); once tools/doku_uret.py")
        return 1
    duvar_doku = [os.path.join(dok, f"duvar_{i}.png") for i in range(a.doku_sayisi)]
    zemin_doku = os.path.join(dok, "zemin.png")

    H, P = a.yukseklik, a.panel
    parcalar = [BASLIK.format(ad=ad, mu=a.mu, ambient=a.ambient, gunes=a.gunes)]

    # --- duvarlar: her tipte var, cunku duvarsiz acik alanda LiDAR'a yapi
    #     kalmiyor ve hicbir SLAM algoritmasi konum kestiremiyor ---
    nx = int(round((L + pay) / P))
    for i in range(nx):
        x = -pay + (i + 0.5) * P
        for yan, y in (("sol", W / 2 + 0.1), ("sag", -W / 2 - 0.1)):
            parcalar.append(kutu(f"duvar_{yan}_{i}", round(x, 3), round(y, 3), H / 2,
                                 P, 0.2, H, duvar_doku[i % len(duvar_doku)]))
    parcalar.append(kutu("duvar_uc", L + 0.1, 0.0, H / 2, 0.2, W + 0.4, H,
                         duvar_doku[0]))
    if a.tip == "oda":      # odanin arka duvari da kapali
        parcalar.append(kutu("duvar_arka", -pay - 0.1, 0.0, H / 2, 0.2, W + 0.4, H,
                             duvar_doku[1 % len(duvar_doku)]))

    # --- zemin ---
    if a.tip == "engebe":
        from PIL import Image
        h_m = ENGEBE_SEVIYELERI[a.seviye]
        boyut = max(L, W) * 3.0
        merkez_x = L / 2.0
        png = os.path.join(acik, "worlds", f"{ad}_yukseklik.png")
        normal = os.path.join(dok, "duz_normal.png")
        if not os.path.exists(normal):
            Image.fromarray(np.tile(np.array([128, 128, 255], np.uint8), (64, 64, 1))
                            ).save(normal)
        if h_m > 0:
            Image.fromarray(yukseklik_resmi(h_m, a.tohum, a.baslangic_x,
                                            merkez_x, boyut), mode="L").save(png)
            parcalar.append(HEIGHTMAP.format(cx=merkez_x, png=png, boyut=boyut,
                                             h=h_m, mu=a.mu, doku=zemin_doku,
                                             normal=normal))
    else:
        nz = int(round((L + pay) / 2.0))
        for i in range(nz):
            parcalar.append(kutu(f"zemin_{i}", round(-pay + (i + 0.5) * 2.0, 3), 0.0,
                                 0.01, 2.0, W, 0.02, zemin_doku, carpisma=False))

    # --- engeller: ROTADAN uzak dur ---
    # Projede otonom gezinme yok (rapor §3); robot sabit rotayi izliyor. Bu
    # yuzden dagniklik SENSORE gorunur olmali ama rotayi KAPATMAMALI. Ilk
    # surumde yalnizca orta serit korunuyordu ve robot her kosuda ayni
    # engele carpiyordu; artik engelin en yakin kosesi rota cizgisine
    # serbest_serit/2 + yari_en kadar uzakta tutuluyor.
    if n_engel:
        import sys as _sys
        _sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        from rota_surucu import rota_uret
        noktalar = [(0.0, 0.0)] + rota_uret(rota_adi)
        rng = np.random.default_rng(a.tohum)
        kenar_x, kenar_y = L - 0.4, W / 2 - 0.4
        yerlesen = 0
        for deneme in range(n_engel * 60):
            if yerlesen >= n_engel:
                break
            kx = float(rng.uniform(0.25, 0.6)); ky = float(rng.uniform(0.25, 0.6))
            kz = float(rng.uniform(0.3, 0.95))
            yari = max(kx, ky) * 0.70721          # dondurulmus kutunun en kotu yarim eni
            ex = float(rng.uniform(-pay + 0.4, kenar_x))
            ey = float(rng.uniform(-kenar_y, kenar_y))
            gerek = a.serbest_serit / 2.0 + yari + 0.15
            if min(nokta_dogru_uzakligi(ex, ey, noktalar[k], noktalar[k + 1])
                   for k in range(len(noktalar) - 1)) < gerek:
                continue
            # dogma bolgesi (0,0) her zaman bos kalmali
            if np.hypot(ex, ey) < 1.0 + yari:
                continue
            parcalar.append(kutu(f"engel_{yerlesen}", round(ex, 3), round(ey, 3),
                                 round(kz / 2, 3), round(kx, 3), round(ky, 3),
                                 round(kz, 3),
                                 duvar_doku[(yerlesen + 3) % len(duvar_doku)],
                                 yaw=round(float(rng.uniform(0, 1.57)), 3)))
            yerlesen += 1
        if yerlesen < n_engel:
            print(f"UYARI: {n_engel} engel istendi, {yerlesen} yerlesti "
                  f"(rota cevresi dar; --serbest-serit dusurulebilir)")
        n_engel = yerlesen

    parcalar.append("  </world>\n</sdf>\n")
    cikti = a.cikti or os.path.join(acik, "worlds", f"{ad}.sdf")
    with open(cikti, "w") as f:
        f.write("".join(parcalar))
    print(f"yazildi: {cikti}")
    ek = (f", engebe seviye {a.seviye} (±{ENGEBE_SEVIYELERI[a.seviye]*50:.0f} cm)"
          if a.tip == "engebe" else "")
    print(f"  tip {a.tip}: {L} x {W} m, {2*nx+1} duvar paneli, {n_engel} engel, "
          f"surtunme mu={a.mu}{ek}")
    print(f"  engelsiz rota: {rota_adi}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
