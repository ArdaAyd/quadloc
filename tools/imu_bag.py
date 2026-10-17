#!/usr/bin/env python3
"""Kayitli bir bag'deki IMU mesajlarina MEMS hata modeli uygular, yeni bag yazar.

    tools/imu_bag.py <kaynak> <hedef> --seviye s3
    tools/imu_bag.py <kaynak> <hedef> --gyro-bias 300 --gyro-yogunluk 0.05
    tools/imu_bag.py <kaynak> <hedef> --seviye s2 --odom-cikar

Duman boru hattiyla ayni mantik: tek temiz kayittan butun seviyeler.

--odom-cikar nedir
------------------
Bacak odometrisi (/odom/raw) kayit sirasinda TEMIZ IMU ile hesaplandi; bu
dosyadaki IMU'yu bozmak onu geriye donuk degistirmez. Bacak odometrisinin de
bozulmus IMU'yu gormesi gerekiyorsa /odom/raw bag'den cikarilir ve bag
oynatilirken state_estimation dugumu canli kostururarak yeniden uretilir.
SLAM algoritmasi dogrudan /imu/data tuketiyorsa (FAST-LIO2, ORB-SLAM3) buna
gerek yoktur.
"""
import argparse, os, sys
import numpy as np
from rosbag2_py import (SequentialReader, SequentialWriter, StorageOptions,
                        ConverterOptions, TopicMetadata)
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import Imu

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from imu_modeli import ImuBozucu, SEVIYELER, SEVIYE_ACIKLAMA, yonelim_boz, quat_to_rpy

CIKARILABILIR = ("/odom/raw", "/odom/filtered", "/odometry/filtered")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("kaynak")
    ap.add_argument("hedef")
    ap.add_argument("--seviye", choices=sorted(SEVIYELER), default=None,
                    help="hazir seviye; tek tek parametrelerle ezilebilir")
    ap.add_argument("--topic", default="/imu/data")
    ap.add_argument("--gyro-yogunluk", type=float, default=None, help="deg/s/sqrt(Hz)")
    ap.add_argument("--gyro-bias", type=float, default=None, help="deg/h")
    ap.add_argument("--ivme-yogunluk", type=float, default=None, help="ug/sqrt(Hz)")
    ap.add_argument("--ivme-bias", type=float, default=None, help="mg")
    ap.add_argument("--olcek", type=float, default=None, help="% sapma")
    ap.add_argument("--tau", type=float, default=100.0, help="bias korelasyon suresi (s)")
    ap.add_argument("--acilis-carpan", type=float, default=1.0,
                    help="acilis kalibrasyonundan artan bias / surus-ici bias")
    ap.add_argument("--kovaryans", choices=("guncelle", "sabit"), default="guncelle",
                    help="guncelle: bozulmus gurultu kovaryansa yansitilir "
                         "(filtre dogru agirlik verir). sabit: sensor kalitesi "
                         "hakkinda yanlis bilgi verir (kalibrasyonsuz durum).")
    ap.add_argument("--yonelim", choices=("boz", "koru"), default="boz",
                    help="orientation alanini da boz (varsayilan) ya da dokunma")
    ap.add_argument("--odom-cikar", action="store_true",
                    help="bacak/EKF odometrisini bag'den cikar (yeniden uretilmeli)")
    ap.add_argument("--tohum", type=int, default=0)
    a = ap.parse_args()

    if a.seviye is None and all(getattr(a, k) is None for k in
                                ("gyro_yogunluk", "gyro_bias", "ivme_yogunluk",
                                 "ivme_bias", "olcek")):
        print("HATA: --seviye ya da en az bir parametre verilmeli"); return 1
    if os.path.exists(a.hedef):
        print(f"HATA: {a.hedef} zaten var"); return 1

    # --- 1. gecis: IMU'yu topla (hata izi tum kosu boyunca tutarli olmali) ---
    okuyucu = SequentialReader()
    okuyucu.open(StorageOptions(uri=a.kaynak, storage_id="mcap"),
                 ConverterOptions("cdr", "cdr"))
    tipler = {t.name: t.type for t in okuyucu.get_all_topics_and_types()}
    if a.topic not in tipler:
        print(f"HATA: '{a.topic}' bag'de yok. Mevcut: {sorted(tipler)}"); return 1

    gyro, ivme, quat, damga = [], [], [], []
    while okuyucu.has_next():
        ad, veri, _ = okuyucu.read_next()
        if ad != a.topic:
            continue
        m = deserialize_message(veri, Imu)
        gyro.append([m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z])
        ivme.append([m.linear_acceleration.x, m.linear_acceleration.y,
                     m.linear_acceleration.z])
        quat.append([m.orientation.x, m.orientation.y, m.orientation.z,
                     m.orientation.w])
        damga.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
    gyro = np.array(gyro); ivme = np.array(ivme)
    quat = np.array(quat); damga = np.array(damga)
    n = len(gyro)
    if n == 0:
        print("HATA: IMU mesaji bulunamadi"); return 1

    sure = float(damga[-1] - damga[0])
    hiz = (n - 1) / sure if sure > 0 else 100.0
    print(f"{n} IMU mesaji, {sure:.1f} s sim suresi, {hiz:.1f} Hz")

    bozucu = ImuBozucu(seviye=a.seviye, hiz_hz=hiz, tau_s=a.tau,
                       acilis_carpan=a.acilis_carpan, tohum=a.tohum,
                       gyro_yogunluk=a.gyro_yogunluk, gyro_bias=a.gyro_bias,
                       ivme_yogunluk=a.ivme_yogunluk, ivme_bias=a.ivme_bias,
                       olcek=a.olcek)
    etiket = f"{a.seviye} ({SEVIYE_ACIKLAMA[a.seviye]})" if a.seviye else "ozel"
    print(f"seviye {etiket}")
    print("  " + bozucu.ozet())

    gyro_b, ivme_b, d_rpy = bozucu.uygula(gyro, ivme, damga)

    # --- 2. gecis: yaz ---
    okuyucu = SequentialReader()
    okuyucu.open(StorageOptions(uri=a.kaynak, storage_id="mcap"),
                 ConverterOptions("cdr", "cdr"))
    cikar = set(CIKARILABILIR) if a.odom_cikar else set()
    yazici = SequentialWriter()
    yazici.open(StorageOptions(uri=a.hedef, storage_id="mcap"),
                ConverterOptions("cdr", "cdr"))
    for ad, tip in tipler.items():
        if ad in cikar:
            print(f"  cikarildi: {ad} (yeniden uretilmeli)")
            continue
        yazici.create_topic(TopicMetadata(id=0, name=ad, type=tip,
                                          serialization_format="cdr"))

    # bozulmus gurultunun kovaryansi (beyaz gurultu + bias kararli durumu)
    g_var = bozucu.sigma_g ** 2 + bozucu.bias_g ** 2
    a_var = bozucu.sigma_a ** 2 + bozucu.bias_a ** 2

    i = 0
    while okuyucu.has_next():
        ad, veri, zaman = okuyucu.read_next()
        if ad in cikar:
            continue
        if ad != a.topic:
            yazici.write(ad, veri, zaman)
            continue
        m = deserialize_message(veri, Imu)
        m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = gyro_b[i]
        m.linear_acceleration.x, m.linear_acceleration.y, \
            m.linear_acceleration.z = ivme_b[i]
        if a.yonelim == "boz":
            q = yonelim_boz(quat[i], d_rpy[i])
            m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w = q
        if a.kovaryans == "guncelle":
            m.angular_velocity_covariance = [
                g_var if k in (0, 4, 8) else 0.0 for k in range(9)]
            m.linear_acceleration_covariance = [
                a_var if k in (0, 4, 8) else 0.0 for k in range(9)]
            if a.yonelim == "boz":
                rp_var = (bozucu.bias_a / 9.80665) ** 2
                yaw_var = float(np.var(d_rpy[:, 2])) or 1e-6
                m.orientation_covariance = [rp_var, 0, 0, 0, rp_var, 0,
                                            0, 0, yaw_var]
        yazici.write(ad, serialize_message(m), zaman)
        i += 1

    # --- ozet ---
    son = d_rpy[-1]
    print(f"{i} IMU mesaji bozuldu")
    print(f"  gyro sapma artisi : {np.linalg.norm(gyro_b - gyro, axis=1).std():.4f} rad/s")
    print(f"  ivme sapma artisi : {np.linalg.norm(ivme_b - ivme, axis=1).std():.4f} m/s^2")
    print(f"  yonelim hatasi (son): roll {np.degrees(son[0]):+.2f} "
          f"pitch {np.degrees(son[1]):+.2f} yaw {np.degrees(son[2]):+.2f} derece")
    print(f"  yaw suruklenmesi |maks| : {np.degrees(np.abs(d_rpy[:,2]).max()):.2f} derece")
    print("yazildi:", a.hedef)
    return 0


if __name__ == "__main__":
    sys.exit(main())
