#!/bin/bash
# 1.2 — temas kaynagi ve bacak odometrisi kampanyasi.
#
#   tools/temas_kampanyasi.sh [tekrar]
#
# Uc yapilandirma x iki hiz x N tekrar, ayrica 0.3 m/s'de N tekrar (A ile).
#   A  sensor, esik kapali       (min=0.0, rel=5.0, topic=foot_contacts)
#   B  sensor, histerezis 15/5 N (min=15.0, rel=5.0)
#   C  yuruyus fazi              (topic=foot_contacts/gait)
HERE=$(cd "$(dirname "$0")" && pwd)
KOK=$HERE/..
# Olcum araci rosbag2_py kullaniyor; ROS burada da kaynaklanmali.
source /opt/ros/jazzy/setup.bash
source $KOK/ws/install/setup.bash
N=${1:-3}
CIKTI=$KOK/sonuclar/12_temas
mkdir -p $CIKTI

kosu() {  # $1=etiket $2=hiz $3=tekrar $4...=--set argumanlari
  local ET=$1 HIZ=$2 T=$3; shift 3
  local AD="12_${ET}_${HIZ}_r${T}"
  local JS=$CIKTI/${AD}.json
  [ -s "$JS" ] && { echo ">>> $AD zaten var, atlaniyor"; return 0; }
  echo "=== $AD ==="
  rm -rf $KOK/kayitlar/$AD
  RAPOR=/tmp/rota_$AD.json $HERE/champ_kosu.sh --kayit $AD \
      --set rota.hiz_ms=$HIZ --set kosu.tohum=$T "$@" > /tmp/kosu_$AD.log 2>&1
  local KOD=$?
  grep -E "^>>> yol|rtf " /tmp/kosu_$AD.log | head -3
  if [ $KOD -ne 0 ] || [ ! -d "$KOK/kayitlar/$AD" ]; then
    echo ">>> $AD BASARISIZ (kod $KOD)"
    echo "{\"bag\":\"$AD\",\"hata\":\"kosu basarisiz, kod $KOD\"}" > $JS
    return 1
  fi
  python3 $HERE/bacak_odom_olc.py $KOK/kayitlar/$AD --cfg $KOK/config/deney_v0.yaml \
      2> /tmp/olc_$AD.err > $JS
  if [ ! -s "$JS" ]; then
    echo ">>> $AD OLCUM BASARISIZ:"; tail -3 /tmp/olc_$AD.err
    echo "{\"bag\":\"$AD\",\"hata\":\"olcum basarisiz\"}" > $JS
    return 1
  fi
  python3 - "$JS" "$ET" "$HIZ" "$T" "$KOK/kayitlar/$AD" <<'PYEOF'
import json, sys, pathlib
js, et, hiz, t, bag = sys.argv[1:6]
d = json.load(open(js))
d.update({"yapilandirma": et, "hiz_ms": float(hiz), "tekrar": int(t)})
rtf = pathlib.Path(bag, "kosu_bilgisi", "rtf.txt")
if rtf.exists():
    for s in rtf.read_text().splitlines():
        p = s.split()
        if len(p) == 2: d[p[0]] = p[1]
json.dump(d, open(js, "w"), ensure_ascii=False, indent=2)
a = d.get("ate", {}).get("rmse", "-")
print(f"    temas={d.get('temas_orani_ortalama','-')} adim={d.get('adim_frekansi_hz','-')} Hz "
      f"ATE={a} son_hata_oran={d.get('son_hata_yol_orani','-')}")
PYEOF
  # Kayitlar SILINMIYOR: 1.6/1.7 icin gerekli ve olcum tekrarlanabilir olmali.
}

for T in $(seq 1 $N); do
  for HIZ in 0.1 0.2; do
    kosu A $HIZ $T --set temas.min_normal_force=0.0  --set temas.contact_release_force=5.0 --set temas.contact_source_topic=foot_contacts
    kosu B $HIZ $T --set temas.min_normal_force=15.0 --set temas.contact_release_force=5.0 --set temas.contact_source_topic=foot_contacts
    kosu C $HIZ $T --set temas.contact_source_topic=foot_contacts/gait
  done
done
# 0.3 m/s dogrulamasi (A ile)
for T in $(seq 1 $N); do
  kosu A 0.3 $T --set temas.min_normal_force=0.0 --set temas.contact_release_force=5.0 --set temas.contact_source_topic=foot_contacts
done
echo; echo "=== json sayisi: $(ls $CIKTI/*.json 2>/dev/null | wc -l) ==="
