#!/bin/bash
# Tek deney: gecerli bir kosu elde edene kadar tekrarlar.
#
#   deney.sh <ad> <sure_sim_s> <ly> <dunya> <profil> [en_fazla_deneme]
#
# Gecersiz kosular (devrilme, takilma, kurulum hatasi) atilir ve yenisi kosulur.
# Her deneme — basarisizlar dahil — kayitlar/kampanya_gunluk.csv'ye yazilir;
# devrilme orani rapor edilebilsin diye bu gunluk korunmali.
#
# Cikis: 0 gecerli kosu elde edildi, 1 en_fazla_deneme tukendi.
AD=${1:?ad gerekli}; SURE=${2:-45}; LY=${3:-1.0}
DUNYA=${4:-warehouse}; PROFIL=${5:-lidar}; MAKS=${6:-4}

HERE=$(cd "$(dirname "$0")" && pwd)
BAGDIR=$HERE/../kayitlar
GUNLUK=$BAGDIR/kampanya_gunluk.csv
mkdir -p $BAGDIR
[ -f "$GUNLUK" ] || echo "zaman,ad,deneme,dunya,profil,ly,sonuc,mesafe_m,hiz_ms,sure_sim_s,rtf,maks_roll_rad,min_z_m,not" > $GUNLUK

for ((i=1; i<=MAKS; i++)); do
  echo "=== $AD — deneme $i/$MAKS ==="
  RAPOR=$(mktemp /tmp/kosu_rapor_XXXX.json)
  RAPOR=$RAPOR $HERE/kosu.sh $SURE $LY $DUNYA "${AD}" $PROFIL
  KOD=$?

  python3 - "$RAPOR" "$GUNLUK" "$AD" "$i" "$DUNYA" "$PROFIL" "$LY" <<'PYEOF'
import json, sys, csv, datetime, os
rapor, gunluk, ad, deneme, dunya, profil, ly = sys.argv[1:8]
d = {}
if os.path.exists(rapor):
    try: d = json.load(open(rapor))
    except Exception: pass
with open(gunluk, "a", newline="") as f:
    csv.writer(f).writerow([
        datetime.datetime.now().isoformat(timespec="seconds"), ad, deneme, dunya, profil, ly,
        d.get("sonuc", "bilinmiyor"), d.get("mesafe_m", ""), d.get("hiz_ms", ""),
        d.get("sure_sim_s", ""), d.get("rtf", ""), d.get("maks_roll_rad", ""),
        d.get("min_govde_z_m", ""), d.get("not", ""),
    ])
PYEOF

  if [ $KOD -eq 0 ]; then
    echo ">>> $AD gecerli (deneme $i)"
    # gecerli kosunun raporunu kaydin yanina koy
    [ -d "$BAGDIR/$AD" ] && cp "$RAPOR" "$BAGDIR/$AD/kosu_raporu.json"
    rm -f "$RAPOR"
    exit 0
  fi

  case $KOD in
    2) SEBEP="devrildi" ;;
    3) SEBEP="takildi" ;;
    4) SEBEP="kurulum hatasi" ;;
    *) SEBEP="bilinmeyen kod $KOD" ;;
  esac
  echo ">>> $AD gecersiz ($SEBEP) — kayit siliniyor"
  rm -rf "$BAGDIR/$AD" "$RAPOR"
done

echo ">>> $AD: $MAKS denemede gecerli kosu elde edilemedi"
exit 1
