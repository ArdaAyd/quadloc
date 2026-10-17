#!/usr/bin/env python3
"""Bir rosbag'deki odometri topic'ini evo'nun TUM bicimine cevirir.

    python3 tools/bag_to_tum.py <bag_dizini> [topic] [cikti.txt]

TUM bicimi: timestamp tx ty tz qx qy qz qw  (bosluk ayrilmis, saniye cinsinden)
Varsayilan topic /ground_truth/odom, yani referans yorunge. SLAM ciktilarini da
ayni betikle cevirip `evo_ape tum ref.txt slam.txt` ile karsilastirabilirsiniz.
"""
import sys, os
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

def main():
    if len(sys.argv) < 2:
        print(__doc__); return 1
    bag = sys.argv[1]
    # CHAMP paketi /odom/ground_truth kullaniyor; eski paket
    # /ground_truth/odom kullaniyordu. Bag'de hangisi varsa o secilir.
    topic = sys.argv[2] if len(sys.argv) > 2 else None
    out = sys.argv[3] if len(sys.argv) > 3 else os.path.join(bag, "ground_truth_tum.txt")

    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag, storage_id="mcap"),
                ConverterOptions("cdr", "cdr"))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic is None:
        for aday in ("/odom/ground_truth", "/ground_truth/odom"):
            if aday in types:
                topic = aday; break
        else:
            print(f"HATA: referans odometri topic'i bulunamadi. Mevcut: {sorted(types)}")
            return 1
    if topic not in types:
        print(f"HATA: '{topic}' bag'de yok. Mevcut: {sorted(types)}"); return 1
    msg_cls = get_message(types[topic])

    n = 0
    with open(out, "w") as f:
        while reader.has_next():
            tname, data, _ = reader.read_next()
            if tname != topic:
                continue
            m = deserialize_message(data, msg_cls)
            # Mesajin kendi zaman damgasini kullan (bag alim zamanini degil):
            # SLAM ciktilariyla hizalanmasi gereken sey bu.
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            p = m.pose.pose.position
            q = m.pose.pose.orientation
            f.write(f"{t:.9f} {p.x:.6f} {p.y:.6f} {p.z:.6f} "
                    f"{q.x:.6f} {q.y:.6f} {q.z:.6f} {q.w:.6f}\n")
            n += 1
    print(f"{n} poz yazildi -> {out}")
    return 0 if n else 1

if __name__ == "__main__":
    sys.exit(main())
