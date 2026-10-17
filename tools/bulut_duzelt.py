#!/usr/bin/env python3
"""LiDAR bulutlarini FAST-LIO'nun velodyne yoluna uygun hale getirir.

    tools/bulut_duzelt.py <kaynak_bag> <hedef_bag>

Iki sorun cozuluyor:

1. ZAMAN ALANI YOK. Gazebo'nun gpu_lidar'i x, y, z, intensity, ring uretiyor;
   'time' alani yok. FAST-LIO bulutu velodyne_ros::Point yapisina donusturuyor
   ve PCL "Failed to find match for field 'time'" deyip alani DOLDURMUYOR —
   degerler cop kaliyor. FAST-LIO de `son_nokta.time > 0` testiyle "ofset
   zamani verilmis" sanip cop zamanlarla hareket duzeltmesi yapiyor ve
   kestirim daginiyor (olculdu: ATE 372 m, yorunge 2194 m).

   Alan SIFIRLARLA ekleniyor. O zaman FAST-LIO `given_offset_time = false`
   yoluna giriyor ve ofset zamani azimut acisindan kendisi turetiyor; bu yol
   iyi sinanmis. Sifir dogru deger cunku simule LiDAR GLOBAL DEKLANSOR:
   butun noktalar ayni anda aliniyor, taramanin icinde duzeltilecek hareket
   yok.

2. SONSUZ NOKTALAR. Donus alinamayan isinlar inf olarak geliyor. Bunlar
   menzil suzgecini geciyor ve bazi taramalarda bulut tamamen kullanilamaz
   hale geliyor ("No point, skip this scan!").

ring ve intensity korunuyor; FAST-LIO halka numarasini katman indeksi olarak
kullaniyor.
"""
import argparse, os, sys
import numpy as np
from rosbag2_py import (SequentialReader, SequentialWriter, StorageOptions,
                        ConverterOptions, TopicMetadata)
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2


def velodyne_bulut(sablon, xyz, intensity, ring, zaman):
    """x,y,z,intensity,time,ring duzeninde PointCloud2 uretir."""
    n = len(xyz)
    veri = np.zeros(n, dtype=np.dtype([
        ("x", np.float32), ("y", np.float32), ("z", np.float32),
        ("intensity", np.float32), ("time", np.float32), ("ring", np.uint16)]))
    veri["x"], veri["y"], veri["z"] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    veri["intensity"] = intensity
    veri["time"] = zaman
    veri["ring"] = ring
    m = PointCloud2()
    m.header = sablon.header
    m.height = 1
    m.width = n
    m.is_dense = True
    m.is_bigendian = False
    m.point_step = veri.dtype.itemsize
    m.row_step = m.point_step * n
    m.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        PointField(name="time", offset=16, datatype=PointField.FLOAT32, count=1),
        PointField(name="ring", offset=20, datatype=PointField.UINT16, count=1),
    ]
    m.data = veri.tobytes()
    return m


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("kaynak")
    ap.add_argument("hedef")
    ap.add_argument("--topic", default="/velodyne_points/points")
    ap.add_argument("--zaman", choices=("sabit", "sifir"), default="sabit",
                    help="sabit: butun noktalara ayni (kucuk pozitif) zaman -> "
                         "FAST-LIO 'ofset verilmis' sayar ve hareket duzeltmesi "
                         "YAPMAZ; simule global deklansor sensor icin dogru olan "
                         "bu. sifir: FAST-LIO ofseti azimuttan TURETIR ve olmayan "
                         "bir bozulmayi duzeltmeye calisir.")
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

    # "sabit" icin kucuk pozitif deger: FAST-LIO son noktanin time > 0 olmasina
    # bakarak ofset zamaninin verildigine karar veriyor; butun noktalar ayni
    # degeri tasidigi icin aralarinda bagil kayma olmuyor, yani duzeltme yok.
    zaman_degeri = 1e-6 if a.zaman == "sabit" else 0.0
    n_bulut = 0; toplam = 0; atilan = 0; bos = 0
    while okuyucu.has_next():
        ad, veri, zaman = okuyucu.read_next()
        if ad != a.topic:
            yazici.write(ad, veri, zaman); continue
        m = deserialize_message(veri, PointCloud2)
        alanlar = [f.name for f in m.fields]
        istenen = ["x", "y", "z"]
        if "intensity" in alanlar: istenen.append("intensity")
        if "ring" in alanlar: istenen.append("ring")
        kayit = list(point_cloud2.read_points(m, field_names=tuple(istenen),
                                              skip_nans=False))
        if not kayit:
            yazici.write(ad, veri, zaman); bos += 1; n_bulut += 1; continue
        dizi = np.array([[float(v) for v in k] for k in kayit], dtype=np.float64)
        xyz = dizi[:, :3]
        sonlu = np.all(np.isfinite(xyz), axis=1)
        toplam += len(xyz); atilan += int(np.sum(~sonlu))
        xyz = xyz[sonlu].astype(np.float32)
        i = 4 if "intensity" in istenen else None
        rr = istenen.index("ring") if "ring" in istenen else None
        yog = dizi[sonlu, istenen.index("intensity")].astype(np.float32) \
            if "intensity" in istenen else np.zeros(len(xyz), np.float32)
        hal = dizi[sonlu, rr].astype(np.uint16) if rr is not None \
            else np.zeros(len(xyz), np.uint16)
        if len(xyz) == 0:
            bos += 1
        yazici.write(ad, serialize_message(
            velodyne_bulut(m, xyz, yog, hal,
                           np.full(len(xyz), zaman_degeri, np.float32))), zaman)
        n_bulut += 1

    print(f"{n_bulut} bulut islendi")
    print(f"  sonsuz nokta atildi: %{100*atilan/max(toplam,1):.1f} "
          f"({atilan}/{toplam})")
    print(f"  bos kalan bulut    : {bos}")
    print(f"  eklenen alan       : time = {zaman_degeri:g} ({a.zaman})")
    print("yazildi:", a.hedef)
    return 0


if __name__ == "__main__":
    sys.exit(main())
