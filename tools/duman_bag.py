#!/usr/bin/env python3
"""Kayitli bir bag'deki LiDAR bulutlarina duman uygular, yeni bag yazar.

    tools/duman_bag.py <kaynak_bag> <hedef_bag> --gorus 20
    tools/duman_bag.py <kaynak_bag> <hedef_bag> --gorus 20 --topic /velodyne_points/points

Tek temiz kayittan istenildigi kadar duman seviyesi uretilebilir. Bu,
simulatorde duman kosturmaya gore uc avantaj sagliyor:
  - seviye surekli ve fiziksel (gorus mesafesi), simulatordeki doyma yok
  - render maliyeti olmadigi icin gercek zaman carpani degismiyor; simulatorde
    duman acikken RTF 0.865'ten 0.830'a dusup robotun yuruyusunu degistiriyordu
  - deterministik: ayni tohum ayni sonucu verir

Duman disindaki butun topicler oldugu gibi kopyalanir.
"""
import argparse, os, sys
import numpy as np
from rosbag2_py import (SequentialReader, SequentialWriter, StorageOptions,
                        ConverterOptions, TopicMetadata)
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs_py import point_cloud2
from sensor_msgs.msg import PointCloud2

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from duman_modeli import duman_uygula, sonum_katsayisi
# Cikti bulutu, bulut_duzelt.py ile AYNI bicimde yaziliyor: x, y, z,
# intensity, time, ring. Ilk surum yalnizca x,y,z,intensity yaziyordu ve
# RING alanini DUSURUYORDU; FAST-LIO'nun velodyne yolu halka numarasini
# katman indeksi olarak kullandigi icin dumanli bag'ler ona verilemiyordu.
# Sonsuz noktalar ve zaman alani da burada hallediliyor, boylece dumanli
# bag ayrica bulut_duzelt'ten gecirilmeden dogrudan SLAM'e verilebiliyor.
from bulut_duzelt import velodyne_bulut


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("kaynak")
    ap.add_argument("hedef")
    ap.add_argument("--gorus", type=float, required=True, help="gorus mesafesi (m)")
    ap.add_argument("--topic", default="/velodyne_points/points")
    ap.add_argument("--tohum", type=int, default=0)
    a = ap.parse_args()

    if os.path.exists(a.hedef):
        print(f"HATA: {a.hedef} zaten var"); return 1

    okuyucu = SequentialReader()
    okuyucu.open(StorageOptions(uri=a.kaynak, storage_id="mcap"),
                 ConverterOptions("cdr", "cdr"))
    tipler = {t.name: t.type for t in okuyucu.get_all_topics_and_types()}
    if a.topic not in tipler:
        print(f"HATA: '{a.topic}' bag'de yok. Mevcut: {sorted(tipler)}"); return 1

    yazici = SequentialWriter()
    yazici.open(StorageOptions(uri=a.hedef, storage_id="mcap"),
                ConverterOptions("cdr", "cdr"))
    for ad, tip in tipler.items():
        yazici.create_topic(TopicMetadata(id=0, name=ad, type=tip,
                                          serialization_format="cdr"))

    cls = get_message(tipler[a.topic])
    alpha = sonum_katsayisi(a.gorus)
    print(f"gorus {a.gorus} m -> sonum katsayisi {alpha:.4f} 1/m")

    n_bulut = 0
    oranlar = {"degismedi": 0, "duman": 0, "silindi": 0}
    while okuyucu.has_next():
        ad, veri, zaman = okuyucu.read_next()
        if ad != a.topic:
            yazici.write(ad, veri, zaman)
            continue
        m = deserialize_message(veri, cls)
        alanlar = [f.name for f in m.fields]
        istenen = ["x", "y", "z"]
        if "intensity" in alanlar: istenen.append("intensity")
        if "ring" in alanlar: istenen.append("ring")
        kayit = list(point_cloud2.read_points(m, field_names=tuple(istenen),
                                              skip_nans=False))
        if not kayit:
            yazici.write(ad, veri, zaman); n_bulut += 1; continue
        dizi = np.array([[float(v) for v in k] for k in kayit], dtype=np.float64)

        # Sonsuz noktalari ONCE ayikla: duman modeli de kendi icinde ayikliyor
        # ama o zaman dondurdugu etiket dizisi ring ile hizasini kaybediyor.
        sonlu = np.all(np.isfinite(dizi[:, :3]), axis=1)
        dizi = dizi[sonlu]
        if len(dizi) == 0:
            yazici.write(ad, veri, zaman); n_bulut += 1; continue

        halka = dizi[:, istenen.index("ring")].astype(np.uint16) \
            if "ring" in istenen else np.zeros(len(dizi), np.uint16)
        p = dizi[:, :4] if "intensity" in istenen else dizi[:, :3]

        # Gazebo'nun gpu_lidar'i bu yapilandirmada intensity alanini DOLDURMUYOR;
        # hepsi sifir geliyor. Sifir yansima "algilanamaz" demek oldugu icin model
        # butun noktalari siliyordu. Alan bos ise varsayilan yansima kullanilir.
        if p.shape[1] > 3 and not np.any(p[:, 3] > 0):
            if n_bulut == 0:
                print("  not: intensity alani bos, varsayilan yansima (1.0) kullaniliyor")
            p = p[:, :3]

        yeni, etiket = duman_uygula(p, a.gorus, tohum=a.tohum + n_bulut)
        oranlar["degismedi"] += int(np.sum(etiket == 0))
        oranlar["duman"] += int(np.sum(etiket == 1))
        oranlar["silindi"] += int(np.sum(etiket == 2))
        kalan = etiket != 2
        yazici.write(ad, serialize_message(velodyne_bulut(
            m, yeni[:, :3].astype(np.float32), yeni[:, 3].astype(np.float32),
            halka[kalan], np.full(len(yeni), 1e-6, np.float32))), zaman)
        n_bulut += 1

    toplam = sum(oranlar.values()) or 1
    print(f"{n_bulut} bulut islendi")
    for k, v in oranlar.items():
        print(f"  {k:<10}: %{100*v/toplam:5.1f}")
    print("yazildi:", a.hedef)
    return 0


if __name__ == "__main__":
    sys.exit(main())
