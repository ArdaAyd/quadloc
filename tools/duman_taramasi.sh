#!/bin/bash
# Duman seviyelerini tarar: her gorus mesafesi icin bozulmus bag uretir,
# SLAM kosturur ve ATE/RPE hesaplar.
#
#   tools/duman_taramasi.sh <temiz_bag> <algoritma> [gorus listesi...]
#   tools/duman_taramasi.sh kayitlar/referans_temiz fastlio temiz 100 50 20 10 5
#
# "temiz" ozel bir seviye: duman uygulanmadan, yalnizca bulut duzeltmesiyle.
# Ara bag'ler SLAM kostuktan sonra siliniyor (her biri ~3 GB).
HERE=$(cd "$(dirname "$0")" && pwd)
KOK=$HERE/..
KAYNAK=${1:?temiz bag gerekli}
ALGO=${2:-fastlio}
shift 2 2>/dev/null
SEVIYELER=${@:-temiz 100 50 20 10 5}
EVO=$KOK/.venv-evo/bin
CIKTI=$KOK/sonuclar/duman_taramasi.csv

source /opt/ros/jazzy/setup.bash
source $KOK/ws/install/setup.bash
mkdir -p $KOK/sonuclar
[ -f "$CIKTI" ] || echo "zaman,kaynak,algoritma,gorus_m,ate_rmse_m,ate_ortalama_m,ate_maks_m,rpe_ceviri_rmse_m,rpe_donus_rmse_derece,slam_poz,ref_poz,not" > $CIKTI

for S in $SEVIYELER; do
  echo "=== gorus = $S ==="
  BAG=$KOK/kayitlar/_tarama_$S
  AD=duman_${S}_${ALGO}
  rm -rf $BAG

  if [ "$S" = "temiz" ]; then
    python3 $HERE/bulut_duzelt.py "$KAYNAK" "$BAG" > /tmp/tarama_bag.log 2>&1
  else
    python3 $HERE/duman_bag.py "$KAYNAK" "$BAG" --gorus $S > /tmp/tarama_bag.log 2>&1
  fi
  if [ ! -d "$BAG" ]; then
    echo ">>> $S: bag uretilemedi"; tail -3 /tmp/tarama_bag.log
    continue
  fi

  $HERE/slam_kosu.sh $ALGO "$BAG" "$AD" > /tmp/tarama_slam.log 2>&1
  KOD=$?
  D=$KOK/sonuclar/$AD
  rm -rf "$BAG"          # ara bag ~3 GB, sonuc cikarildi

  if [ $KOD -ne 0 ] || [ ! -s "$D/slam_tum.txt" ]; then
    echo ">>> $S: SLAM basarisiz (kod $KOD)"
    echo "$(date -Is),$KAYNAK,$ALGO,$S,,,,,,,,SLAM basarisiz" >> $CIKTI
    continue
  fi

  python3 - "$D" "$EVO" "$CIKTI" "$KAYNAK" "$ALGO" "$S" <<'PYEOF'
import subprocess, sys, csv, datetime, re, os
d, evo, cikti, kaynak, algo, seviye = sys.argv[1:7]
def olc(cmd):
    try:
        c = subprocess.run(cmd, capture_output=True, text=True, timeout=300)
        g = {}
        for satir in c.stdout.splitlines():
            m = re.match(r"\s*(rmse|mean|max)\s+([\d.eE+-]+)", satir)
            if m: g[m.group(1)] = float(m.group(2))
        return g
    except Exception:
        return {}
ref, slam = f"{d}/referans_tum.txt", f"{d}/slam_tum.txt"
ortak = ["--align", "--t_max_diff", "0.05"]
ate = olc([f"{evo}/evo_ape", "tum", ref, slam] + ortak)
rpe_t = olc([f"{evo}/evo_rpe", "tum", ref, slam, "--delta", "1",
             "--delta_unit", "m", "--t_max_diff", "0.05"])
rpe_r = olc([f"{evo}/evo_rpe", "tum", ref, slam, "--delta", "1",
             "--delta_unit", "m", "--pose_relation", "angle_deg",
             "--t_max_diff", "0.05"])
n_s = sum(1 for _ in open(slam)); n_r = sum(1 for _ in open(ref))
with open(cikti, "a", newline="") as f:
    csv.writer(f).writerow([
        datetime.datetime.now().isoformat(timespec="seconds"), kaynak, algo, seviye,
        ate.get("rmse",""), ate.get("mean",""), ate.get("max",""),
        rpe_t.get("rmse",""), rpe_r.get("rmse",""), n_s, n_r, ""])
print(f">>> {seviye}: ATE {ate.get('rmse','-')} m  "
      f"RPE_t {rpe_t.get('rmse','-')} m  RPE_r {rpe_r.get('rmse','-')} deg  "
      f"({n_s} poz)")
PYEOF
done

echo; echo "=== ozet ==="; column -s, -t $CIKTI
