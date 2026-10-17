#!/usr/bin/env python3
"""Kaydedilmis kosudan animasyon ve ozet grafik uretir.

    python3 tools/goster.py <bag_dizini> [cikti_onadi]

Ureticileri: <onek>_animasyon.mp4 (ustten gorunum, LiDAR taramasi + yorunge)
             <onek>_ozet.png      (yorunge, govde yuksekligi, roll/pitch, hiz)
"""
import sys, os, math
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs_py import point_cloud2

GT = "/ground_truth/odom"
SCAN = "/scan/points"

def quat_rpy(q):
    roll = math.atan2(2*(q.w*q.x+q.y*q.z), 1-2*(q.x*q.x+q.y*q.y))
    pitch = math.asin(max(-1, min(1, 2*(q.w*q.y-q.z*q.x))))
    yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    return roll, pitch, yaw

def read(bag):
    r = SequentialReader()
    r.open(StorageOptions(uri=bag, storage_id="mcap"), ConverterOptions("cdr","cdr"))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    cls = {n: get_message(t) for n,t in types.items() if n in (GT, SCAN)}
    poses, scans = [], []
    while r.has_next():
        name, data, _ = r.read_next()
        if name == GT:
            m = deserialize_message(data, cls[GT])
            t = m.header.stamp.sec + m.header.stamp.nanosec*1e-9
            p, o = m.pose.pose.position, m.pose.pose.orientation
            v = m.twist.twist.linear
            poses.append((t, p.x, p.y, p.z, *quat_rpy(o), math.hypot(v.x, v.y)))
        elif name == SCAN:
            m = deserialize_message(data, cls[SCAN])
            t = m.header.stamp.sec + m.header.stamp.nanosec*1e-9
            pts = point_cloud2.read_points_numpy(m, field_names=("x","y","z"), skip_nans=True)
            scans.append((t, pts))
    return np.array(poses), scans

def main():
    bag = sys.argv[1]
    pre = sys.argv[2] if len(sys.argv) > 2 else os.path.join(bag, "kosu")
    P, scans = read(bag)
    if len(P) == 0:
        print("odometri yok"); return 1
    t0 = P[0,0]
    P[:,0] -= t0
    scans = [(t-t0, p) for t,p in scans]
    print(f"{len(P)} poz, {len(scans)} LiDAR taramasi, sure {P[-1,0]:.1f} s")

    # --- ozet grafik ---
    fig, ax = plt.subplots(2, 2, figsize=(13, 7.5))
    ax[0,0].plot(P[:,1], P[:,2], lw=2, color="#1f77b4")
    ax[0,0].plot(P[0,1], P[0,2], "o", color="green", ms=9, label="baslangic")
    ax[0,0].plot(P[-1,1], P[-1,2], "s", color="red", ms=9, label="bitis")
    ax[0,0].set_title("Yorunge (ustten, referans konum)"); ax[0,0].set_xlabel("x [m]")
    ax[0,0].set_ylabel("y [m]"); ax[0,0].axis("equal"); ax[0,0].grid(alpha=.3); ax[0,0].legend()

    ax[0,1].plot(P[:,0], P[:,3], color="#ff7f0e")
    ax[0,1].axhline(0.34, ls="--", c="gray", lw=1, label="nominal 0.34 m")
    ax[0,1].set_title("Govde yuksekligi"); ax[0,1].set_xlabel("t [s]")
    ax[0,1].set_ylabel("z [m]"); ax[0,1].grid(alpha=.3); ax[0,1].legend()

    ax[1,0].plot(P[:,0], np.degrees(P[:,4]), label="roll")
    ax[1,0].plot(P[:,0], np.degrees(P[:,5]), label="pitch")
    ax[1,0].plot(P[:,0], np.degrees(P[:,6]), label="yaw", alpha=.6)
    ax[1,0].set_title("Govde acilari"); ax[1,0].set_xlabel("t [s]")
    ax[1,0].set_ylabel("derece"); ax[1,0].grid(alpha=.3); ax[1,0].legend()

    ax[1,1].plot(P[:,0], P[:,7], color="#2ca02c")
    ax[1,1].axhline(0.40, ls="--", c="gray", lw=1, label="komut 0.40 m/s")
    ax[1,1].set_title("Yatay hiz"); ax[1,1].set_xlabel("t [s]")
    ax[1,1].set_ylabel("m/s"); ax[1,1].grid(alpha=.3); ax[1,1].legend()
    fig.suptitle("Go2 / Gazebo Harmonic — yurume kosusu", fontsize=13)
    fig.tight_layout(); fig.savefig(pre+"_ozet.png", dpi=110); plt.close(fig)
    print("yazildi:", pre+"_ozet.png")

    # --- animasyon: ustten gorunum, LiDAR taramasi govde cercevesinden dunyaya tasinir ---
    fig, a = plt.subplots(figsize=(10, 6))
    pad = 3
    a.set_xlim(P[:,1].min()-pad, P[:,1].max()+pad)
    a.set_ylim(P[:,2].min()-pad, P[:,2].max()+pad)
    a.set_aspect("equal"); a.grid(alpha=.25)
    a.set_xlabel("x [m]"); a.set_ylabel("y [m]")
    harita, = a.plot([], [], ".", ms=.6, color="#b0b0b0", alpha=.5)
    tarama, = a.plot([], [], ".", ms=1.4, color="#d62728")
    iz,     = a.plot([], [], "-", lw=2, color="#1f77b4")
    govde,  = a.plot([], [], "o", ms=10, color="#1f77b4", mec="k")
    yon,    = a.plot([], [], "-", lw=2.5, color="black")
    baslik  = a.set_title("")

    birikmis_x, birikmis_y = [], []
    w = FFMpegWriter(fps=12, bitrate=2400)
    with w.saving(fig, pre+"_animasyon.mp4", dpi=110):
        for t, pts in scans:
            i = int(np.argmin(np.abs(P[:,0]-t)))
            x, y, z, roll, pitch, yaw, v = P[i,1:8]
            if len(pts):
                p = np.asarray(pts, dtype=float).reshape(-1, 3)
                # yalnizca govde seviyesine yakin noktalar: ustten gorunumu okunakli tutar
                p = p[np.abs(p[:,2]) < 1.5]
                c, s = math.cos(yaw), math.sin(yaw)
                wx = x + c*p[:,0] - s*p[:,1]
                wy = y + s*p[:,0] + c*p[:,1]
                tarama.set_data(wx, wy)
                birikmis_x.extend(wx[::7]); birikmis_y.extend(wy[::7])
                harita.set_data(birikmis_x, birikmis_y)
            iz.set_data(P[:i+1,1], P[:i+1,2])
            govde.set_data([x], [y])
            yon.set_data([x, x+0.6*math.cos(yaw)], [y, y+0.6*math.sin(yaw)])
            baslik.set_text(f"t = {t:5.1f} s    hiz = {v:4.2f} m/s    "
                            f"govde = {z:4.2f} m    roll = {math.degrees(roll):+5.1f}°")
            w.grab_frame()
    plt.close(fig)
    print("yazildi:", pre+"_animasyon.mp4")
    return 0

if __name__ == "__main__":
    sys.exit(main())
