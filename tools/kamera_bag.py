#!/usr/bin/env python3
"""Kayitli bir bag'deki kamera goruntulerine karanlik ve/veya duman uygular.

    tools/kamera_bag.py <kaynak> <hedef> --karanlik k3
    tools/kamera_bag.py <kaynak> <hedef> --gorus 5
    tools/kamera_bag.py <kaynak> <hedef> --karanlik k2 --gorus 10   # ikisi birden

Duman ve IMU boru hatlariyla ayni mantik: tek temiz kayittan butun seviyeler.

Sira: once DUMAN (sahne isiginin sacilmasi, kameraya girmeden once olur),
sonra KARANLIK (aydinlatma + sensor zinciri). Dumanin kendi parlakligi ortam
isigiyla olceklendigi icin bu sira dogru: karanlik odada duman da karanliktir.

Derinlik goruntusu de duman uygulandiginda bozulur (dusuk kontrastta stereo
eslestirme coker). Karanlik derinligi bozmaz: D455 derinligi aktif IR
projektorle uretir, ortam isigina bagli degildir — gercek donanimin davranisi
budur ve raporda ayrica belirtilmelidir.
"""
import argparse, os, sys
import numpy as np
from rosbag2_py import (SequentialReader, SequentialWriter, StorageOptions,
                        ConverterOptions, TopicMetadata)
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import Image

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from kamera_modeli import (karanlik_uygula, kamera_duman_uygula, orb_sayisi,
                           rms_kontrast, KARANLIK_SEVIYELERI, KARANLIK_ACIKLAMA)


def damga(m):
    return m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec


def goruntu_al(m):
    if m.encoding == "rgb8":
        return np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3), 3
    if m.encoding == "bgr8":
        a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)
        return a[:, :, ::-1].copy(), 3
    if m.encoding == "mono8":
        return np.frombuffer(m.data, np.uint8).reshape(m.height, m.width), 1
    raise ValueError(f"desteklenmeyen kodlama: {m.encoding}")


def goruntu_yaz(m, dizi):
    if m.encoding == "bgr8":
        dizi = dizi[:, :, ::-1]
    m.data = np.ascontiguousarray(dizi).tobytes()
    return m


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("kaynak")
    ap.add_argument("hedef")
    ap.add_argument("--karanlik", choices=sorted(KARANLIK_SEVIYELERI), default=None)
    ap.add_argument("--lux", type=float, default=None,
                    help="seviye yerine dogrudan aydinlik duzeyi")
    ap.add_argument("--gorus", type=float, default=None,
                    help="kamera dumani icin gorus mesafesi (m); LiDAR modeliyle ayni")
    ap.add_argument("--atmosfer", type=float, default=None,
                    help="dumanin dogrusal parlakligi (0..1); verilmezse "
                         "her kareden kestirilir")
    ap.add_argument("--maks-kazanc", type=float, default=32.0,
                    help="otomatik kazanc tavani")
    ap.add_argument("--goruntu-topic", default="/d455/image")
    ap.add_argument("--derinlik-topic", default="/d455/depth_image")
    ap.add_argument("--metrik", action="store_true",
                    help="ORB kose sayisi ve kontrast olc (yavaslatir)")
    ap.add_argument("--tohum", type=int, default=0)
    a = ap.parse_args()

    lux = a.lux if a.lux is not None else (
        KARANLIK_SEVIYELERI[a.karanlik] if a.karanlik else None)
    if lux is None and a.gorus is None:
        print("HATA: --karanlik/--lux ya da --gorus verilmeli"); return 1
    if os.path.exists(a.hedef):
        print(f"HATA: {a.hedef} zaten var"); return 1

    okuyucu = SequentialReader()
    okuyucu.open(StorageOptions(uri=a.kaynak, storage_id="mcap"),
                 ConverterOptions("cdr", "cdr"))
    tipler = {t.name: t.type for t in okuyucu.get_all_topics_and_types()}
    if a.goruntu_topic not in tipler:
        print(f"HATA: '{a.goruntu_topic}' bag'de yok. Mevcut: {sorted(tipler)}")
        return 1
    duman_var = a.gorus is not None
    if duman_var and a.derinlik_topic not in tipler:
        print(f"HATA: duman icin derinlik gerekli, '{a.derinlik_topic}' bag'de yok")
        return 1

    yazici = SequentialWriter()
    yazici.open(StorageOptions(uri=a.hedef, storage_id="mcap"),
                ConverterOptions("cdr", "cdr"))
    for ad, tip in tipler.items():
        yazici.create_topic(TopicMetadata(id=0, name=ad, type=tip,
                                          serialization_format="cdr"))

    if lux is not None:
        et = f"{a.karanlik} ({KARANLIK_ACIKLAMA[a.karanlik]})" if a.karanlik else "ozel"
        print(f"karanlik: {lux:g} lux — {et}, kazanc tavani {a.maks_kazanc:g}x")
    if duman_var:
        at = f"{a.atmosfer:g}" if a.atmosfer is not None else "sahneden kestiriliyor"
        print(f"kamera dumani: gorus {a.gorus:g} m, atmosfer parlakligi {at}")

    rng = np.random.default_rng(a.tohum)
    # Bag'in bastan kesilmis olmasi yuzunden ilk RGB'den once derinlik
    # gelmeyebiliyor (olculdu: bir kayitta ilk mesaj RGB idi ve arac hata
    # verip duruyordu). Bu yuzden RGB, esi gelene kadar SIRA KORUNARAK
    # bekletiliyor; sira bozulursa bag'in oynatma sirasi bozulur.
    derinlikler = {}          # damga -> (float32 dizi, mesaj, zaman)
    n_g = n_d = 0
    eksik_derinlik = 0
    olcum = {"orb_once": [], "orb_sonra": [], "kontrast_once": [],
             "kontrast_sonra": [], "parlaklik_once": [], "parlaklik_sonra": []}
    bilgi_son = {}

    bekleyen = []             # sirali cikis kuyrugu: [topic, veri|None, zaman, damga]

    def rgb_isle(m, zaman, d):
        """Bir RGB karesini (ve varsa esli derinligi) bozar, yazar."""
        nonlocal n_g
        g, _ = goruntu_al(m)
        if a.metrik and n_g % 20 == 0:
            olcum["orb_once"].append(orb_sayisi(g))
            olcum["kontrast_once"].append(rms_kontrast(g))
            olcum["parlaklik_once"].append(float(g.mean()))
        d_yeni = None
        if duman_var and d is not None:
            g, d_yeni, bilgi = kamera_duman_uygula(
                g, d, a.gorus, atmosfer=a.atmosfer, rng=rng)
            bilgi_son.update(bilgi)
        if lux is not None:
            g, bilgi = karanlik_uygula(g, lux, maks_kazanc=a.maks_kazanc, rng=rng)
            bilgi_son.update(bilgi)
        if a.metrik and n_g % 20 == 0:
            olcum["orb_sonra"].append(orb_sayisi(g))
            olcum["kontrast_sonra"].append(rms_kontrast(g))
            olcum["parlaklik_sonra"].append(float(g.mean()))
        n_g += 1
        return serialize_message(goruntu_yaz(m, g)), d_yeni

    def bosalt(hepsi=False):
        """Cozulmus girdileri SIRAYLA yazar."""
        nonlocal eksik_derinlik
        while bekleyen:
            topic, veri, zaman, anahtar = bekleyen[0][:4]
            if veri is None:                      # esi bekleniyor
                if not hepsi:
                    return
                # bag sonunda esi hic gelmedi: oldugu gibi/dumansiz yaz
                m = bekleyen[0][4]
                eksik_derinlik += 1
                if topic == a.derinlik_topic:
                    derinlikler.pop(anahtar, None)
                    veri = serialize_message(m)
                else:
                    veri, _ = rgb_isle(m, zaman, None)
            yazici.write(topic, veri, zaman)
            bekleyen.pop(0)

    while okuyucu.has_next():
        ad, veri, zaman = okuyucu.read_next()

        if ad == a.derinlik_topic and duman_var:
            m = deserialize_message(veri, Image)
            d = np.frombuffer(m.data, np.float32).reshape(m.height, m.width)
            anahtar = damga(m)
            # Bu derinligi bekleyen bir RGB var mi? Varsa ikisini de burada ure.
            bekleyen_rgb = next((g for g in bekleyen
                                 if g[0] == a.goruntu_topic and g[1] is None
                                 and g[3] == anahtar), None)
            if bekleyen_rgb is not None:
                yeni_veri, d_yeni = rgb_isle(bekleyen_rgb[4], bekleyen_rgb[2], d)
                bekleyen_rgb[1] = yeni_veri
                m.data = np.ascontiguousarray(d_yeni, dtype=np.float32).tobytes()
                bekleyen.append([ad, serialize_message(m), zaman, anahtar])
            else:
                derinlikler[anahtar] = (d, m, zaman)
                bekleyen.append([ad, None, zaman, anahtar, m])   # RGB'yi bekler
            n_d += 1
            bosalt()
            continue

        if ad != a.goruntu_topic:
            bekleyen.append([ad, veri, zaman, None])
            bosalt()
            continue

        m = deserialize_message(veri, Image)
        anahtar = damga(m)
        if duman_var:
            d_kayit = derinlikler.pop(anahtar, None)
            if d_kayit is None:
                # derinlik henuz gelmedi: sirayi koruyarak beklet
                bekleyen.append([ad, None, zaman, anahtar, m])
                bosalt()
                continue
            yeni_veri, d_yeni = rgb_isle(m, zaman, d_kayit[0])
            # bekleyen derinlik girdisini doldur
            for g in bekleyen:
                if g[0] == a.derinlik_topic and g[1] is None and g[3] == anahtar:
                    dm = d_kayit[1]
                    dm.data = np.ascontiguousarray(d_yeni, dtype=np.float32).tobytes()
                    g[1] = serialize_message(dm)
                    break
            bekleyen.append([ad, yeni_veri, zaman, anahtar])
        else:
            yeni_veri, _ = rgb_isle(m, zaman, None)
            bekleyen.append([ad, yeni_veri, zaman, anahtar])
        bosalt()

    bosalt(hepsi=True)

    print(f"{n_g} goruntu, {n_d} derinlik islendi")
    if eksik_derinlik:
        print(f"UYARI: {eksik_derinlik} derinlik karesi eslesmedi, oldugu gibi yazildi")
    for k, v in bilgi_son.items():
        print(f"  {k}: {v:.4g}" if isinstance(v, float) else f"  {k}: {v}")
    if a.metrik and olcum["orb_once"]:
        o, s = np.mean(olcum["orb_once"]), np.mean(olcum["orb_sonra"])
        print(f"  ORB kose      : {o:.0f} -> {s:.0f}  (%{100*s/max(o,1e-9):.1f})")
        print(f"  RMS kontrast  : {np.mean(olcum['kontrast_once']):.4f} -> "
              f"{np.mean(olcum['kontrast_sonra']):.4f}")
        print(f"  ort parlaklik : {np.mean(olcum['parlaklik_once']):.1f} -> "
              f"{np.mean(olcum['parlaklik_sonra']):.1f}")
    print("yazildi:", a.hedef)
    return 0


if __name__ == "__main__":
    sys.exit(main())
