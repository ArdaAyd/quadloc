#!/bin/bash
# Tek deney koşusu. BÜTÜN parametreler config/deney_v0.yaml'den gelir;
# launch varsayılanlarına güvenilmez.
#
#   tools/champ_kosu.sh --kayit AD [--cfg config/deney_v0.yaml]
#                       [--set temas.min_normal_force=15.0] [--set ...]
#
# --set ile yapılandırmanın herhangi bir alanı o koşu için geçersiz kılınır;
# kullanılan nihai değerler koşu klasörüne yazılır.
#
# Koşu klasörü (kayitlar/<AD>/kosu_bilgisi/) şunları içerir:
#   commitler.txt     ana depo + alt modül commit'leri
#   yapilandirma.yaml bu koşuda kullanılan nihai değerler
#   launch_komutu.txt birebir çalıştırılan komut
#   param_dump/       bütün düğümlerin ros2 param dump çıktısı
#   param_dogrulama.txt  yapılandırma ile param dump karşılaştırması
#   rtf.txt           gerçek zaman çarpanı (duvar saati / sim saati)
#
# ROS setup betikleri tanımsız değişken kullanıyor, bu yüzden "set -u" yok.
HERE=$(cd "$(dirname "$0")" && pwd)
KOK=$HERE/..
WS=$KOK/ws
BAGDIR=$KOK/kayitlar

CFG=$KOK/config/deney_v0.yaml
KAYIT=; declare -a SETS=()
while [ $# -gt 0 ]; do
  case $1 in
    --cfg)   CFG=$2; shift 2 ;;
    --kayit) KAYIT=$2; shift 2 ;;
    --set)   SETS+=("$2"); shift 2 ;;
    *) echo "bilinmeyen secenek: $1"; exit 1 ;;
  esac
done
[ -f "$CFG" ] || { echo "HATA: yapilandirma yok: $CFG"; exit 1; }

source /opt/ros/jazzy/setup.bash
source $WS/install/setup.bash
export DISPLAY=${DISPLAY:-:0}

# --- nihai yapilandirmayi uret (--set'ler uygulanmis) ---
TMPCFG=$(mktemp /tmp/deneycfg_XXXX.yaml)
python3 - "$CFG" "$TMPCFG" "${SETS[@]}" <<'PYEOF'
import sys, yaml, pathlib
kaynak, hedef, *sets = sys.argv[1:]
d = yaml.safe_load(pathlib.Path(kaynak).read_text())
for s in sets:
    yol, _, deger = s.partition("=")
    p = d
    parcalar = yol.split(".")
    for k in parcalar[:-1]:
        p = p[k]
    eski = p[parcalar[-1]]
    if isinstance(eski, bool):   yeni = deger.lower() in ("1","true","yes")
    elif isinstance(eski, int) and not isinstance(eski, bool): yeni = int(float(deger))
    elif isinstance(eski, float): yeni = float(deger)
    else: yeni = deger
    p[parcalar[-1]] = yeni
pathlib.Path(hedef).write_text(yaml.safe_dump(d, allow_unicode=True, sort_keys=False))
PYEOF
oku() { python3 $HERE/cfg_oku.py "$TMPCFG" "$1"; }

DUNYA=$(oku simulasyon.dunya);           GUI=$(oku simulasyon.gui)
RVIZ=$(oku simulasyon.rviz);             SURTUNME=$(oku simulasyon.foot_friction)
C_TOPIC=$(oku temas.contact_source_topic); C_MIN=$(oku temas.min_normal_force)
C_REL=$(oku temas.contact_release_force);  C_TMO=$(oku temas.contact_timeout)
ODOM_RATE=$(oku odometri.odom_rate);     MAP_ODOM=$(oku tf.publish_map_odom)
ROTA=$(oku rota.ad);                     MUTLAK=$(oku rota.mutlak)
BASLANGIC=$(oku rota.baslangic);         HIZ=$(oku rota.hiz_ms)
PROFIL=$(oku kayit.profil);              DENEME=$(oku kosu.deneme)
TOHUM=$(oku kosu.tohum)
ORTAK=$(oku kayit.ortak_topicler)

case $PROFIL in
  lidar)  KAMERA_KAPAT=true;  LIDAR_KAPAT=false; TOPICS="$ORTAK /velodyne_points/points" ;;
  kamera) KAMERA_KAPAT=false; LIDAR_KAPAT=true;  TOPICS="$ORTAK /d455/image /d455/depth_image /d455/camera_info" ;;
  tumu)   KAMERA_KAPAT=false; LIDAR_KAPAT=false; TOPICS="$ORTAK /velodyne_points/points /d455/image /d455/depth_image /d455/camera_info" ;;
  *) echo "bilinmeyen profil: $PROFIL"; exit 1 ;;
esac
[ "$MUTLAK" = "true" ] && MUTLAK_BAYRAK=--mutlak || MUTLAK_BAYRAK=

DUNYA_YOL=$DUNYA
[ -f "$DUNYA_YOL" ] || DUNYA_YOL=$WS/src/unitree_go2_ros2/unitree_go2_description/worlds/$DUNYA.sdf
[ -f "$DUNYA_YOL" ] || { echo "HATA: dunya bulunamadi: $DUNYA"; exit 1; }

trap '$HERE/temizle.sh >/dev/null 2>&1' EXIT

for DENE in $(seq 1 $DENEME); do
echo "### deneme $DENE/$DENEME"
$HERE/temizle.sh >/dev/null 2>&1

LAUNCH_KOMUTU="ros2 launch unitree_go2_sim unitree_go2_launch.py \
world:=$DUNYA_YOL gui:=$GUI rviz:=$RVIZ \
disable_camera:=$KAMERA_KAPAT disable_velodyne_lidar:=$LIDAR_KAPAT \
foot_friction:=$SURTUNME contact_source_topic:=$C_TOPIC \
min_normal_force:=$C_MIN contact_release_force:=$C_REL \
contact_timeout:=$C_TMO publish_map_odom:=$MAP_ODOM"

echo "### simulasyon (dunya=$DUNYA profil=$PROFIL temas=$C_TOPIC min=$C_MIN rel=$C_REL)"
eval "$LAUNCH_KOMUTU" > /tmp/champ_sim.log 2>&1 &

BEKLENEN="joint_states_controller joint_group_effort_controller"
for i in $(seq 1 180); do
  EKSIK=""
  for c in $BEKLENEN; do
    ros2 control list_controllers 2>/dev/null | grep -q "^$c .*active" || EKSIK="$EKSIK $c"
  done
  [ -z "$EKSIK" ] && break
  sleep 1
done
[ -n "$EKSIK" ] && { echo "HATA: aktif olmayan kontrolcüler:$EKSIK"; exit 4; }
echo "### kontrolcüler aktif"

for t in $TOPICS; do
  [ "$t" = "/tf_static" ] && continue
  for i in $(seq 1 90); do
    ros2 topic info $t 2>/dev/null | grep -q 'Publisher count: [1-9]' && break
    sleep 1
  done
  ros2 topic info $t 2>/dev/null | grep -q 'Publisher count: [1-9]' || echo "UYARI: $t yayinda degil"
done
sleep 3

# Ayak temasi: en az bir ayak temas bildirmeli. Gelmiyorsa bacak odometrisi
# sessizce donar (dunyada gz-sim-contact-system eksikse boyle olur).
TEMAS=0
for t in 1 2 3 4 5; do
  TEMAS=$(timeout 8 ros2 topic echo /foot_contacts --once 2>/dev/null | grep -c "^- true")
  [ "${TEMAS:-0}" -gt 0 ] && break
  sleep 2
done
[ "${TEMAS:-0}" -eq 0 ] && { echo "HATA: hicbir ayak temas bildirmiyor"; exit 4; }
echo "### ayak temasi dogrulandi ($TEMAS ayak)"

# --- kosu bilgisi klasoru ---
BILGI=/tmp/kosu_bilgisi_$$
rm -rf $BILGI; mkdir -p $BILGI/param_dump
cp "$TMPCFG" $BILGI/yapilandirma.yaml
echo "$LAUNCH_KOMUTU" > $BILGI/launch_komutu.txt
{
  echo "ana_depo $(cd $KOK && git rev-parse HEAD 2>/dev/null)"
  for m in ws/src/unitree_go2_ros2 ws/src/FAST_LIO; do
    [ -d "$KOK/$m/.git" ] || [ -f "$KOK/$m/.git" ] && \
      echo "$m $(cd $KOK/$m && git rev-parse HEAD 2>/dev/null)"
  done
  echo "tohum $TOHUM"
} > $BILGI/commitler.txt
for n in $(ros2 node list 2>/dev/null); do
  ad=$(echo "$n" | tr '/' '_' | sed 's/^_//')
  timeout 25 ros2 param dump "$n" > "$BILGI/param_dump/${ad}.yaml" 2>/dev/null
done
python3 $HERE/param_dogrula.py "$TMPCFG" "$BILGI/param_dump" > $BILGI/param_dogrulama.txt 2>&1
PD=$?
tail -3 $BILGI/param_dogrulama.txt
[ $PD -ne 0 ] && echo "UYARI: param dump ile yapilandirma arasinda FARK VAR"

if [ -n "$KAYIT" ]; then
  rm -rf $BAGDIR/$KAYIT; mkdir -p $BAGDIR
  echo "### kayit: $BAGDIR/$KAYIT"
  ros2 bag record -o $BAGDIR/$KAYIT --storage mcap --topics $TOPICS > /tmp/champ_bag.log 2>&1 &
  BAG_PID=$!
  for i in $(seq 1 30); do
    grep -q "All requested topics are subscribed" /tmp/champ_bag.log && break
    sleep 1
  done
fi

DUVAR0=$(date +%s.%N)
SIM0=$(timeout 10 ros2 topic echo /clock --once 2>/dev/null | grep -A1 "sec:" | head -2 | grep -oE "[0-9]+" | head -1)

echo "### rota: $ROTA (hiz $HIZ m/s)"
python3 $HERE/rota_surucu.py --rota=$ROTA --hiz=$HIZ $MUTLAK_BAYRAK \
        --baslangic=$BASLANGIC --json=${RAPOR:-/tmp/champ_rota.json}
DURUM=$?

DUVAR1=$(date +%s.%N)
SIM1=$(timeout 10 ros2 topic echo /clock --once 2>/dev/null | grep -A1 "sec:" | head -2 | grep -oE "[0-9]+" | head -1)
python3 - "$DUVAR0" "$DUVAR1" "${SIM0:-0}" "${SIM1:-0}" > $BILGI/rtf.txt <<'PYEOF'
import sys
d0,d1,s0,s1 = (float(x) for x in sys.argv[1:5])
duvar = d1-d0; sim = s1-s0
print(f"duvar_saati_s {duvar:.2f}")
print(f"sim_saati_s   {sim:.2f}")
print(f"rtf           {sim/duvar:.3f}" if duvar>0 else "rtf           -")
PYEOF
cat $BILGI/rtf.txt

if [ -n "$KAYIT" ]; then
  sleep 2; kill -INT $BAG_PID 2>/dev/null; sleep 4; kill -9 $BAG_PID 2>/dev/null
fi

if [ $DURUM -eq 2 ] && [ $DENE -lt $DENEME ]; then
  echo ">>> devrildi, tekrar deneniyor"
  [ -n "$KAYIT" ] && rm -rf $BAGDIR/$KAYIT
  $HERE/temizle.sh >/dev/null 2>&1; sleep 3; continue
fi

if [ -n "$KAYIT" ] && [ -d "$BAGDIR/$KAYIT" ]; then
  [ -f $BAGDIR/$KAYIT/metadata.yaml ] || ros2 bag reindex $BAGDIR/$KAYIT -s mcap
  rm -rf $BAGDIR/$KAYIT/kosu_bilgisi
  cp -r $BILGI $BAGDIR/$KAYIT/kosu_bilgisi
fi
rm -f "$TMPCFG"
exit $DURUM
done
echo ">>> $DENEME denemede gecerli kosu elde edilemedi"
rm -f "$TMPCFG"
exit 2
