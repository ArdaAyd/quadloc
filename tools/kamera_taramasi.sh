#!/bin/bash
# Kamera kosullarini tarar: karanlik ve/veya kamera dumani seviyeleri icin
# bozulmus bag uretir, gorsel SLAM kosturur, ATE/RPE hesaplar.
#
#   tools/kamera_taramasi.sh <temiz_bag> <algoritma> <kosul> [seviyeler...]
#   tools/kamera_taramasi.sh kayitlar/referans_temiz rtabmap karanlik temiz k1 k2 k3 k4
#   tools/kamera_taramasi.sh kayitlar/referans_temiz rtabmap duman   temiz 100 50 20 10 5
#
# "temiz" ozel seviye: bozulma uygulanmaz, yalnizca gorsel SLAM icin
# cerceve/TF duzeltmesi yapilir.
HERE=$(cd "$(dirname "$0")" && pwd)
KOK=$HERE/..
KAYNAK=${1:?temiz bag gerekli}
ALGO=${2:-rtabmap}
KOSUL=${3:?kosul gerekli (karanlik | duman)}
shift 3 2>/dev/null
case $KOSUL in
  karanlik) VARSAYILAN="temiz k1 k2 k3 k4" ;;
  duman)    VARSAYILAN="temiz 100 50 20 10 5" ;;
  *) echo "bilinmeyen kosul: $KOSUL"; exit 1 ;;
esac
SEVIYELER=${@:-$VARSAYILAN}
EVO=$KOK/.venv-evo/bin
CIKTI=$KOK/sonuclar/kamera_taramasi.csv

source /opt/ros/jazzy/setup.bash
source $KOK/ws/install/setup.bash
mkdir -p $KOK/sonuclar
[ -f "$CIKTI" ] || echo "zaman,kaynak,algoritma,kosul,seviye,ate_rmse_m,ate_ortalama_m,ate_maks_m,rpe_ceviri_rmse_m,rpe_donus_rmse_derece,slam_poz,ref_poz,not" > $CIKTI

for S in $SEVIYELER; do
  echo "=== $KOSUL = $S ==="
  HAM=$KOK/kayitlar/_ktarama_ham_$S
  BAG=$KOK/kayitlar/_ktarama_$S
  AD=${KOSUL}_${S}_${ALGO}
  rm -rf $HAM $BAG

  # 1) kosulu uygula (temiz ise atla)
  if [ "$S" = "temiz" ]; then
    HAM=$KAYNAK
  elif [ "$KOSUL" = "karanlik" ]; then
    python3 $HERE/kamera_bag.py "$KAYNAK" "$HAM" --karanlik $S > /tmp/ktarama_bag.log 2>&1
  else
    python3 $HERE/kamera_bag.py "$KAYNAK" "$HAM" --gorus $S > /tmp/ktarama_bag.log 2>&1
  fi
  if [ ! -d "$HAM" ]; then
    echo ">>> $S: bozulmus bag uretilemedi"; tail -3 /tmp/ktarama_bag.log; continue
  fi

  # 2) gorsel SLAM icin hazirla (optik cerceve, /tf ve map->odom cikarma)
  python3 $HERE/kamera_slam_bag.py "$HAM" "$BAG" > /tmp/ktarama_hazirla.log 2>&1
  [ "$S" = "temiz" ] || rm -rf "$HAM"
  if [ ! -d "$BAG" ]; then
    echo ">>> $S: SLAM bag'i hazirlanamadi"; tail -3 /tmp/ktarama_hazirla.log; continue
  fi

  # 3) SLAM
  $HERE/slam_kosu.sh $ALGO "$BAG" "$AD" > /tmp/ktarama_slam.log 2>&1
  KOD=$?
  D=$KOK/sonuclar/$AD
  rm -rf "$BAG"

  if [ $KOD -ne 0 ] || [ ! -s "$D/slam_tum.txt" ]; then
    echo ">>> $S: SLAM cikti uretmedi (kod $KOD) — gorsel odometri kaybolmus olabilir"
    echo "$(date -Is),$KAYNAK,$ALGO,$KOSUL,$S,,,,,,,,SLAM cikti uretmedi" >> $CIKTI
    continue
  fi

  python3 - "$D" "$EVO" "$CIKTI" "$KAYNAK" "$ALGO" "$KOSUL" "$S" <<'PYEOF'
import subprocess, sys, csv, datetime, re
d, evo, cikti, kaynak, algo, kosul, seviye = sys.argv[1:8]
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
        datetime.datetime.now().isoformat(timespec="seconds"), kaynak, algo,
        kosul, seviye, ate.get("rmse",""), ate.get("mean",""), ate.get("max",""),
        rpe_t.get("rmse",""), rpe_r.get("rmse",""), n_s, n_r, ""])
print(f">>> {seviye}: ATE {ate.get('rmse','-')} m  RPE_t {rpe_t.get('rmse','-')} m  "
      f"({n_s} poz)")
PYEOF
done

echo; echo "=== ozet ==="; column -s, -t $CIKTI
