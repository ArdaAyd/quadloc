#!/usr/bin/env python3
"""Bir koşu kaydından bacak odometrisi metriklerini çıkarır.

    tools/bacak_odom_olc.py <bag_dizini> [--cfg config/deney_v0.yaml]

Ölçülenler (hepsi SIM SAATI, mesaj başlık damgalarından):
  - ayak başına temas oranı ve adım frekansı
  - evo ile ATE ve RPE (Δ=1 m), hizalama SE(3) Umeyama (ölçeksiz)
  - son konum hatası / yol uzunluğu
  - yol uzunluğu oranı (tam hız ve 2 Hz) — eski sayılarla karşılaştırma için

JSON basar.
"""
import argparse, json, math, pathlib, subprocess, sys, tempfile
import numpy as np
import yaml
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def bag_oku(bag, topicler):
    r = SequentialReader()
    r.open(StorageOptions(uri=str(bag), storage_id="mcap"), ConverterOptions("cdr", "cdr"))
    tipler = {t.name: t.type for t in r.get_all_topics_and_types()}
    cikti = {t: [] for t in topicler if t in tipler}
    sinif = {t: get_message(tipler[t]) for t in cikti}
    while r.has_next():
        ad, veri, _ = r.read_next()
        if ad in cikti:
            cikti[ad].append(deserialize_message(veri, sinif[ad]))
    return cikti


def damga(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def tum_yaz(yol, mesajlar):
    with open(yol, "w") as f:
        for m in mesajlar:
            p, q = m.pose.pose.position, m.pose.pose.orientation
            f.write(f"{damga(m):.9f} {p.x:.6f} {p.y:.6f} {p.z:.6f} "
                    f"{q.x:.6f} {q.y:.6f} {q.z:.6f} {q.w:.6f}\n")


def evo_calistir(evo, alt, ref, est, ek):
    try:
        c = subprocess.run([f"{evo}/{alt}", "tum", ref, est] + ek,
                           capture_output=True, text=True, timeout=300)
        g = {}
        for s in c.stdout.splitlines():
            parts = s.split()
            if len(parts) == 2 and parts[0] in ("rmse", "mean", "median", "max", "std"):
                g[parts[0]] = float(parts[1])
        return g or {"hata": c.stderr.strip().splitlines()[-1] if c.stderr else "cikti yok"}
    except Exception as e:
        return {"hata": str(e)}


def yol_uzunlugu(xy):
    return float(np.sum(np.linalg.norm(np.diff(xy, axis=0), axis=1))) if len(xy) > 1 else 0.0


def seyrelt(xy, t, ara):
    out = [xy[0]]; t0 = t[0]
    for p, tt in zip(xy, t):
        if tt - t0 >= ara:
            out.append(p); t0 = tt
    return np.array(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--cfg", default="config/deney_v0.yaml")
    ap.add_argument("--evo", default=None)
    a = ap.parse_args()
    cfg = yaml.safe_load(pathlib.Path(a.cfg).read_text())
    kok = pathlib.Path(a.cfg).resolve().parent.parent
    evo = a.evo or str(kok / ".venv-evo" / "bin")
    if not pathlib.Path(evo, "evo_ape").exists():
        evo = "/opt/venv-evo/bin"

    d = bag_oku(a.bag, ["/odom/ground_truth", "/odom/raw", "/foot_contacts"])
    gt, raw, tem = d.get("/odom/ground_truth", []), d.get("/odom/raw", []), d.get("/foot_contacts", [])
    if not gt or not raw:
        print(json.dumps({"hata": "odometri topicleri eksik"})); return 1

    sonuc = {"bag": str(a.bag), "N_gt": len(gt), "N_raw": len(raw), "saat": "sim"}

    # --- temas orani ve adim frekansi (sim saati) ---
    if tem:
        T = np.array([list(m.contacts) for m in tem], dtype=bool)
        tt = np.array([damga(m) for m in tem])
        sonuc["temas_orani_ayak"] = [round(float(x), 4) for x in T.mean(axis=0)]
        sonuc["temas_orani_ortalama"] = round(float(T.mean()), 4)
        # adim frekansi: bir ayagin temas->havada gecis sayisi / sure
        sure = float(tt[-1] - tt[0]) if len(tt) > 1 else 0.0
        gecis = [int(np.sum(np.diff(T[:, i].astype(int)) == 1)) for i in range(T.shape[1])]
        sonuc["adim_frekansi_hz"] = round(float(np.mean(gecis) / sure), 3) if sure > 0 else None
        sonuc["temas_sure_s"] = round(sure, 2)

    # --- yol uzunluklari ---
    g_xy = np.array([[m.pose.pose.position.x, m.pose.pose.position.y] for m in gt])
    r_xy = np.array([[m.pose.pose.position.x, m.pose.pose.position.y] for m in raw])
    g_t = np.array([damga(m) for m in gt]); r_t = np.array([damga(m) for m in raw])
    sonuc["yol_gt_tam_m"] = round(yol_uzunlugu(g_xy), 3)
    sonuc["yol_raw_tam_m"] = round(yol_uzunlugu(r_xy), 3)
    sonuc["yol_orani_tam"] = round(yol_uzunlugu(r_xy) / max(yol_uzunlugu(g_xy), 1e-9) - 1, 4)
    g2 = seyrelt(g_xy, g_t, 0.5); r2 = seyrelt(r_xy, r_t, 0.5)
    sonuc["yol_gt_2hz_m"] = round(yol_uzunlugu(g2), 3)
    sonuc["yol_raw_2hz_m"] = round(yol_uzunlugu(r2), 3)
    sonuc["yol_orani_2hz"] = round(yol_uzunlugu(r2) / max(yol_uzunlugu(g2), 1e-9) - 1, 4)

    # --- son konum hatasi / yol uzunlugu ---
    # Ikisi de kendi basladiklari yerden olculur (baslangic noktalari ortusur).
    g_son = g_xy[-1] - g_xy[0]; r_son = r_xy[-1] - r_xy[0]
    son_hata = float(np.linalg.norm(g_son - r_son))
    sonuc["son_konum_hatasi_m"] = round(son_hata, 4)
    sonuc["son_hata_yol_orani"] = round(son_hata / max(yol_uzunlugu(g2), 1e-9), 4)

    # --- evo ATE / RPE ---
    with tempfile.TemporaryDirectory() as td:
        ref = f"{td}/ref.txt"; est = f"{td}/est.txt"
        tum_yaz(ref, gt); tum_yaz(est, raw)
        tmd = str(cfg["degerlendirme"]["t_max_diff_s"])
        delta = str(cfg["degerlendirme"]["rpe_delta_m"])
        sonuc["hizalama"] = cfg["degerlendirme"]["hizalama"]
        sonuc["ate"] = evo_calistir(evo, "evo_ape", ref, est, ["--align", "--t_max_diff", tmd])
        sonuc["rpe_ceviri"] = evo_calistir(evo, "evo_rpe", ref, est,
                                           ["--delta", delta, "--delta_unit", "m", "--t_max_diff", tmd])
        sonuc["rpe_donus_derece"] = evo_calistir(evo, "evo_rpe", ref, est,
                                                 ["--delta", delta, "--delta_unit", "m",
                                                  "--pose_relation", "angle_deg", "--t_max_diff", tmd])
    print(json.dumps(sonuc, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
