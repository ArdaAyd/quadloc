#!/usr/bin/env python3
"""Bir kaydi gorsel SLAM'e (RTAB-Map / ORB-SLAM3) verilebilir hale getirir.

    tools/kamera_slam_bag.py <kaynak_bag> <hedef_bag>

Uc duzeltme yapiyor; ucu de yapilmazsa SONUC SESSIZCE YANLIS olur.

1. KAMERA CERCEVESI. Gazebo koprusu goruntulere frame_id olarak
   'd455_camera_link' yaziyor. O cerceve OPTIK DEGIL: z ekseni yukari bakiyor,
   optik kuralda ise z ileri bakmali. RTAB-Map goruntunun cercevesini optik
   kabul ettigi icin kamerayi yukari bakar sanar ve yorunge tamamen yanlis
   cikar. frame_id, TF'te zaten var olan ve dogru kurala sahip
   'd455_camera_color_optical_frame' ile degistiriliyor.

2. ODOMETRI TF CAKISMASI. Kayitta robotun kendi EKF'i 'odom -> base_footprint'
   yayinliyor. RTAB-Map'in rgbd_odometry'si de odom->base yayinlayacak; ayni
   donusumun iki yayincisi olunca TF agaci tutarsizlasir. Gorsel SLAM icin
   robotun eklem/odometri TF'lerine ihtiyac yok (kamera <- base zinciri
   tamamen STATIK), bu yuzden /tf tamamen cikariliyor.

3. map -> odom. Kayitta statik birim donusum olarak var; RTAB-Map bunu
   kendisi yayinlar. tf_static'ten yalnizca o donusum ayiklaniyor, digerleri
   (kamera zinciri, lidar, imu) korunuyor.

Referans yorunge /odom/ground_truth TOPIC'inde oldugu icin /tf'in
cikarilmasi degerlendirmeyi etkilemiyor.
"""
import argparse, os, sys
from rosbag2_py import (SequentialReader, SequentialWriter, StorageOptions,
                        ConverterOptions, TopicMetadata)
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import Image, CameraInfo
from tf2_msgs.msg import TFMessage

OPTIK = "d455_camera_color_optical_frame"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("kaynak")
    ap.add_argument("hedef")
    ap.add_argument("--optik-cerceve", default=OPTIK)
    ap.add_argument("--tf-koru", action="store_true",
                    help="/tf'i cikarma (robotun kendi odometrisi de kalsin)")
    a = ap.parse_args()
    if os.path.exists(a.hedef):
        print(f"HATA: {a.hedef} zaten var"); return 1

    okuyucu = SequentialReader()
    okuyucu.open(StorageOptions(uri=a.kaynak, storage_id="mcap"),
                 ConverterOptions("cdr", "cdr"))
    tipler = {t.name: t.type for t in okuyucu.get_all_topics_and_types()}
    kamera = [t for t in ("/d455/image", "/d455/depth_image", "/d455/camera_info")
              if t in tipler]
    if not kamera:
        print(f"HATA: kamera topic'i yok. Mevcut: {sorted(tipler)}"); return 1

    yazici = SequentialWriter()
    yazici.open(StorageOptions(uri=a.hedef, storage_id="mcap"),
                ConverterOptions("cdr", "cdr"))
    for ad, tip in tipler.items():
        if ad == "/tf" and not a.tf_koru:
            continue
        yazici.create_topic(TopicMetadata(id=0, name=ad, type=tip,
                                          serialization_format="cdr"))

    sayac = {"cerceve": 0, "tf_atlandi": 0, "map_odom_atildi": 0}
    while okuyucu.has_next():
        ad, veri, zaman = okuyucu.read_next()

        if ad == "/tf" and not a.tf_koru:
            sayac["tf_atlandi"] += 1
            continue

        if ad == "/tf_static":
            m = deserialize_message(veri, TFMessage)
            kalan = [t for t in m.transforms
                     if not (t.header.frame_id == "map" and t.child_frame_id == "odom")]
            sayac["map_odom_atildi"] += len(m.transforms) - len(kalan)
            if not kalan:
                continue
            m.transforms = kalan
            yazici.write(ad, serialize_message(m), zaman)
            continue

        if ad in kamera:
            cls = get_message(tipler[ad])
            m = deserialize_message(veri, cls)
            m.header.frame_id = a.optik_cerceve
            sayac["cerceve"] += 1
            yazici.write(ad, serialize_message(m), zaman)
            continue

        yazici.write(ad, veri, zaman)

    print(f"{sayac['cerceve']} kamera mesajinin cercevesi -> {a.optik_cerceve}")
    print(f"{sayac['tf_atlandi']} /tf mesaji cikarildi"
          if not a.tf_koru else "/tf korundu")
    print(f"{sayac['map_odom_atildi']} map->odom donusumu ayiklandi")
    print("yazildi:", a.hedef)
    return 0


if __name__ == "__main__":
    sys.exit(main())
