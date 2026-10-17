#!/usr/bin/env python3
"""CHAMP/Go2'yi tanimli bir rota uzerinde yurutur ve bacak odometrisini olcer.

    rota_surucu.py --rota gidis_donus:8 [--hiz 0.2] [--json CIKTI]

Neden sabit rota: robot serbest yurutuldugunde her kosuda farkli kavis
ciziyor; ayni komutla yol uzunlugu 8.5-9.8 m, hiz 0.21-0.24 m/s arasinda
degisiyordu ve bacak odometrisi hatasi ayni kosulda +%4 ile +%14 arasinda
savruluyordu. Bu varyans, olculmek istenen kayma etkisinden buyuk. Her kosu
ayni yorungeyi izlerse kosullar (surtunme, duman, ...) arasindaki fark
yorunge farkina karismaz.

Rotalar:
    duz:12             ileri 12 m
    gidis_donus:8      ileri 8 m, don, basa gel
    kare:6             6x6 m kare tur
    zigzag:12:2        12 m boyunca 2 m genlikli zikzak

Yonelim duzeltmesi simulatorun referans konumundan yapilir. Bu mesru: otonom
gezinme projenin kapsami disinda (rapor §3); amac yorungenin kosular arasinda
ayni kalmasi.

Olculen metrik YOL UZUNLUGU (integre edilmis), bastan sona duz mesafe degil:
robot kavis cizdiginde duz mesafe bacak odometrisiyle karsilastirilamaz.
"""
import math, sys, time, json
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

SEYRELT_S   = 0.5     # s, seyreltilmis yol icin ornekleme araligi (2 Hz)
DEVRIK_Z    = 0.15    # govde merkezi bunun altindaysa robot yerde
DEVRIK_ACI  = 0.9     # rad, roll/pitch siniri
DEVRIK_SURE = 1.0     # s, bu kadar surekli devrik kalinca kosu gecersiz
KP_YAW      = 1.2     # hedefe yonelme kazanci
KP_YANAL    = 0.4     # yanal sapmanin yonelime katkisi, rad/s per m
KP_YANAL_V  = 0.8     # yanal sapmanin YANAL HIZA katkisi, (m/s) per m
MAKS_VY     = 0.15    # m/s, gait.yaml siniri 0.25
VARIS_M     = 0.45    # hedefe bu kadar yaklasinca sonrakine gec
YAVASLA     = 0.25    # yonelim bozukken ileri hizin taban carpani
MAKS_YAW    = 0.5     # rad/s, CHAMP gait.yaml siniri


def rota_uret(ad):
    p = ad.split(":"); tur = p[0]
    sayi = [float(x) for x in p[1:]] or [10.0]
    m = sayi[0]
    if tur == "duz":          return [(m, 0.0)]
    if tur == "gidis_donus":  return [(m, 0.0), (0.0, 0.0)]
    if tur == "kare":         return [(m, 0.0), (m, m), (0.0, m), (0.0, 0.0)]
    if tur == "zigzag":
        g = sayi[1] if len(sayi) > 1 else 2.0
        a = m / 4.0
        return [(a, g), (2*a, -g), (3*a, g), (m, 0.0)]
    raise ValueError(f"bilinmeyen rota: {ad}")


def sar(a):
    while a >  math.pi: a -= 2*math.pi
    while a < -math.pi: a += 2*math.pi
    return a


class Surucu(Node):
    def __init__(self):
        super().__init__("rota_surucu")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        q = QoSProfile(depth=10); q.reliability = ReliabilityPolicy.BEST_EFFORT
        self.gt = self.raw = None
        self.yaw = 0.0; self.t = None
        self.z = None; self.roll = 0.0; self.pitch = 0.0
        self.devrik_sure = 0.0; self._son_t = None
        self.yol_gt = 0.0; self.yol_raw = 0.0; self.olc = False
        self._g = self._r = None
        # Seyreltilmis yol: tam hizda (50 Hz) olculen yolun ~%24'u ADIM
        # SALINIMI, gercek ilerleme degil (olculdu: 16.63 m -> 2 Hz'de 12.73 m).
        # Yuruyusu degistiren kosullarda (surtunme, engebe) salinim da
        # degistigi icin tam hizdaki yol, kaymayi salinimla karistiriyor.
        # 2 Hz'de yol, gidilen MESAFEYI temsil ediyor.
        self.yol_gt_s = 0.0; self.yol_raw_s = 0.0
        self._gs = self._rs = None
        self._t_gs = self._t_rs = None
        self.ilk_gt = self.son_gt = None
        self.create_subscription(Odometry, "/odom/ground_truth", self._on_gt, q)
        self.create_subscription(Odometry, "/odom/raw", self._on_raw, 10)

    def _on_gt(self, m):
        p = m.pose.pose.position; o = m.pose.pose.orientation
        yeni = (p.x, p.y)
        self.yaw = math.atan2(2*(o.w*o.z + o.x*o.y), 1 - 2*(o.y*o.y + o.z*o.z))
        self.roll = math.atan2(2*(o.w*o.x + o.y*o.z), 1 - 2*(o.x*o.x + o.y*o.y))
        self.pitch = math.asin(max(-1.0, min(1.0, 2*(o.w*o.y - o.z*o.x))))
        self.z = p.z
        t_yeni = m.header.stamp.sec + m.header.stamp.nanosec*1e-9
        # Devrilme takibi: govde cokmus ya da asiri yatmis halde gecen sure.
        # Devrilme tespiti olmadan dusen robot sure sinirina kadar bekleniyor
        # ve GB'larca kullanilamaz kayit uretiliyor (olculdu: 1.8 GB).
        if self._son_t is not None:
            dt = max(0.0, t_yeni - self._son_t)
            devrik = (p.z < DEVRIK_Z) or (abs(self.roll) > DEVRIK_ACI) \
                     or (abs(self.pitch) > DEVRIK_ACI)
            self.devrik_sure = (self.devrik_sure + dt) if devrik else 0.0
        self._son_t = t_yeni
        self.t = t_yeni
        if self.olc and self._g:
            self.yol_gt += math.hypot(yeni[0]-self._g[0], yeni[1]-self._g[1])
            if self.ilk_gt is None:
                self.ilk_gt = yeni
            self.son_gt = yeni
            if self._t_gs is None or t_yeni - self._t_gs >= SEYRELT_S:
                if self._gs is not None:
                    self.yol_gt_s += math.hypot(yeni[0]-self._gs[0], yeni[1]-self._gs[1])
                self._gs = yeni; self._t_gs = t_yeni
        self._g = yeni; self.gt = yeni

    def _on_raw(self, m):
        p = m.pose.pose.position; yeni = (p.x, p.y)
        t_yeni = m.header.stamp.sec + m.header.stamp.nanosec*1e-9
        if self.olc and self._r:
            self.yol_raw += math.hypot(yeni[0]-self._r[0], yeni[1]-self._r[1])
            if self._t_rs is None or t_yeni - self._t_rs >= SEYRELT_S:
                if self._rs is not None:
                    self.yol_raw_s += math.hypot(yeni[0]-self._rs[0], yeni[1]-self._rs[1])
                self._rs = yeni; self._t_rs = t_yeni
        self._r = yeni; self.raw = yeni

    def spin(self, s):
        t0 = time.time()
        while time.time()-t0 < s:
            rclpy.spin_once(self, timeout_sec=0.005)

    def sur(self, vx, wz, vy=0.0):
        c = Twist(); c.linear.x = vx; c.linear.y = vy; c.angular.z = wz
        self.pub.publish(c)

    def dur(self):
        for _ in range(20):
            self.sur(0.0, 0.0); self.spin(0.05)


def main():
    rota_adi = "gidis_donus:8"; hiz = 0.2; json_yolu = None; isinma = 10.0
    mutlak = False; baslangic = None
    for a in sys.argv[1:]:
        if a.startswith("--rota="):  rota_adi = a.split("=",1)[1]
        elif a.startswith("--hiz="): hiz = float(a.split("=",1)[1])
        elif a.startswith("--json="): json_yolu = a.split("=",1)[1]
        elif a.startswith("--isinma="): isinma = float(a.split("=",1)[1])
        elif a == "--mutlak": mutlak = True
        elif a.startswith("--baslangic="):
            baslangic = tuple(float(v) for v in a.split("=",1)[1].split(","))

    rclpy.init(); d = Surucu(); d.spin(4.0)
    sonuc = {"rota": rota_adi, "komut_hiz_ms": hiz}

    def bitir(kod, not_=""):
        sonuc["not"] = not_
        print("SONUC_JSON " + json.dumps(sonuc, ensure_ascii=False), flush=True)
        if json_yolu:
            json.dump(sonuc, open(json_yolu, "w"), indent=2, ensure_ascii=False)
        rclpy.shutdown(); sys.exit(kod)

    if d.gt is None or d.raw is None:
        bitir(4, "/odom/ground_truth veya /odom/raw gelmiyor")

    # Isinma: yuruyus oturmadan olcume baslamak sonuclari bozuyor
    t0 = d.t
    while d.t - t0 < isinma:
        d.sur(hiz, 0.0); d.spin(0.02)
        if d.devrik_sure > DEVRIK_SURE:
            d.dur()
            bitir(2, f"devrildi (isinmada, z={d.z:.2f} roll={d.roll:+.2f} "
                     f"pitch={d.pitch:+.2f})")

    # Sabit olcum baslangici. Isinma nereye goturursa olcum oradan baslarsa
    # ayni rotada olculen yol kosudan kosuya degisir (olculdu: duz:6 rotasinda
    # 3.1 - 5.5 m) ve yuzdeler karsilastirilamaz hale gelir. --baslangic
    # verilince robot once oraya SURULUR (olcumsuz), olcum oradan baslar.
    if baslangic is not None:
        t_b = d.t
        while math.hypot(baslangic[0]-d.gt[0], baslangic[1]-d.gt[1]) > VARIS_M:
            hb = sar(math.atan2(baslangic[1]-d.gt[1], baslangic[0]-d.gt[0]) - d.yaw)
            d.sur(hiz * max(YAVASLA, math.cos(hb)),
                  max(-MAKS_YAW, min(MAKS_YAW, KP_YAW * hb))); d.spin(0.02)
            if d.devrik_sure > DEVRIK_SURE:
                d.dur(); bitir(2, f"devrildi (baslangica giderken, z={d.z:.2f})")
            if d.t - t_b > 180.0:
                d.dur(); bitir(3, f"olcum baslangicina ({baslangic}) ulasilamadi")
        print(f"-> olcum baslangici {baslangic} ulasildi (t={d.t-t_b:.0f} s)", flush=True)

    noktalar = rota_uret(rota_adi)
    if mutlak:
        # Rota noktalari DUNYA cercevesinde. Uretilen dunyalarda engeller
        # rotanin etrafinda bosluk birakacak sekilde yerlestiriliyor; rota
        # robotun isinma sonrasi konumuna gore kurulursa iki taraf ayni
        # rotayi bilmez ve bosluklar ~2 m kayar.
        hedefler = list(noktalar)
        bx, by = 0.0, 0.0
    else:
        c, s_ = math.cos(d.yaw), math.sin(d.yaw)
        bx, by = d.gt
        hedefler = [(bx + c*x - s_*y, by + s_*x + c*y) for x, y in noktalar]
    basla = d.gt if mutlak else (bx, by)
    beklenen = sum(math.hypot(b[0]-a[0], b[1]-a[1])
                   for a, b in zip([basla] + hedefler, hedefler))
    print(f"-> rota '{rota_adi}': {len(hedefler)} hedef, beklenen yol {beklenen:.1f} m", flush=True)

    d.yol_gt = d.yol_raw = 0.0; d.olc = True
    i = 0; t1 = d.t; sure_siniri = beklenen / max(hiz*0.4, 0.02) + 60.0
    onceki = basla
    while i < len(hedefler):
        hx, hy = hedefler[i]
        dx, dy = hx - d.gt[0], hy - d.gt[1]
        uzak = math.hypot(dx, dy)
        # Yalnizca yaricapa bakmak yetmiyor: robot hedefi yandan gecerse (dar
        # koridorda yana kayma olur) bir daha yaricapa girmiyor ve hedefin
        # etrafinda daire cizerek sureyi tuketiyor (olculdu: 6 m'lik rotada
        # 23.5 m yol). Bu yuzden GECIS de varis sayilir: yaklasma yonundeki
        # izdusum hedefi astiysa hedefe ulasilmis kabul edilir.
        ax, ay = hx - onceki[0], hy - onceki[1]
        bacak = math.hypot(ax, ay)
        gecti = False
        if bacak > 1e-6:
            izdusum = ((d.gt[0]-onceki[0])*ax + (d.gt[1]-onceki[1])*ay) / bacak
            gecti = izdusum > bacak
        if uzak < VARIS_M or gecti:
            i += 1
            onceki = (hx, hy)
            print(f"   hedef {i}/{len(hedefler)} "
                  f"{'gecildi' if gecti and uzak >= VARIS_M else 'ulasildi'}"
                  f"  (t={d.t-t1:.0f} s, uzaklik {uzak:.2f} m)", flush=True)
            continue
        # Yalnizca hedefe yonelme hatasina bakmak yetmiyor: hedef uzaktayken
        # aci hatasi kucuk kaliyor ve robot yanal kaymayi toparlayamiyor
        # (olculdu: 5 m'de 1.5 m sola kayip engele carpti). Bu yuzden
        # hedefe-yonelme + YANAL SAPMA birlikte kullaniliyor; yanal sapma
        # yoruungeyi kosular arasinda da tekrarlanabilir kiliyor.
        hata = sar(math.atan2(dy, dx) - d.yaw)
        yanal = 0.0
        if bacak > 1e-6:
            # pozitif = robot dogru parcasinin SOLUNDA
            yanal = (ax*(d.gt[1]-onceki[1]) - ay*(d.gt[0]-onceki[0])) / bacak
        wz = max(-MAKS_YAW, min(MAKS_YAW, KP_YAW * hata - KP_YANAL * yanal))
        # CHAMP holonomik: robot yana da yurüyebiliyor (gait.yaml
        # max_linear_velocity_y = 0.25). Yanal kaymayi yalnizca donusle
        # duzeltmek yavas kaliyordu — robot dar koridorda 7 m'de 1.7 m sola
        # kayip ilerleyemiyordu. Dogrudan yanal hiz vermek sapmayi aninda
        # kapatiyor ve yorungeyi kosular arasinda tekrarlanabilir kiliyor.
        vy = max(-MAKS_VY, min(MAKS_VY, -KP_YANAL_V * yanal))
        vx = hiz * max(YAVASLA, math.cos(hata))
        d.sur(vx, wz, vy); d.spin(0.02)
        if d.devrik_sure > DEVRIK_SURE:
            d.olc = False; d.dur()
            bitir(2, f"devrildi ({i}/{len(hedefler)} hedef, z={d.z:.2f} "
                     f"roll={d.roll:+.2f} pitch={d.pitch:+.2f})")
        if d.t - t1 > sure_siniri:
            d.olc = False; d.dur()
            bitir(3, f"rota tamamlanamadi: {i}/{len(hedefler)} hedef, sure doldu")
    d.olc = False

    sure = d.t - t1
    hata_y = 100*(d.yol_raw/max(d.yol_gt,1e-6) - 1)
    hata_s = 100*(d.yol_raw_s/max(d.yol_gt_s,1e-6) - 1)
    yer_degis = (math.hypot(d.son_gt[0]-d.ilk_gt[0], d.son_gt[1]-d.ilk_gt[1])
                 if d.ilk_gt and d.son_gt else 0.0)
    sonuc.update({
        "sure_sim_s": round(sure, 2),
        "yol_gercek_m": round(d.yol_gt, 3),
        "yol_bacak_odom_m": round(d.yol_raw, 3),
        "bacak_odom_hatasi_yuzde": round(hata_y, 1),
        # Seyreltilmis (2 Hz) yol: adim salinimi ayiklanmis, gercek ilerleme
        "yol_gercek_seyreltik_m": round(d.yol_gt_s, 3),
        "yol_bacak_odom_seyreltik_m": round(d.yol_raw_s, 3),
        "bacak_odom_hatasi_seyreltik_yuzde": round(hata_s, 1),
        "salinim_payi_yuzde": round(100*(1 - d.yol_gt_s/max(d.yol_gt,1e-6)), 1),
        "yer_degistirme_m": round(yer_degis, 3),
        "ortalama_hiz_ms": round(d.yol_gt_s/max(sure,1e-6), 3),
        "beklenen_yol_m": round(beklenen, 2),
    })
    print(f"\n>>> yol (tam hiz): gercek {d.yol_gt:.2f} m | bacak odom {d.yol_raw:.2f} m "
          f"| hata {hata_y:+.1f} %", flush=True)
    print(f">>> yol (2 Hz)   : gercek {d.yol_gt_s:.2f} m | bacak odom {d.yol_raw_s:.2f} m "
          f"| hata {hata_s:+.1f} %   (salinim payi %{sonuc['salinim_payi_yuzde']:.0f})",
          flush=True)
    print(f">>> sure {sure:.1f} s sim, ortalama hiz {d.yol_gt/max(sure,1e-6):.3f} m/s", flush=True)
    d.dur()
    bitir(0)


if __name__ == "__main__":
    main()
