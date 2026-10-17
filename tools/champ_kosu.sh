#!/bin/bash
# CHAMP paketi (unitree_go2_ros2) icin tek deney kosusu.
#
#   champ_kosu.sh --rota gidis_donus:8 [--dunya bos] [--kayit AD]
#                 [--profil lidar|kamera|tumu] [--hiz 0.2] [--gui]
#
# kosu.sh digger paket (gz_quadruped_playground, /control_input) icindi.
# Bu paket /cmd_vel kullanir ve topic adlari farklidir, bu yuzden ayri betik.
#
# ROS setup betikleri tanimsiz degisken kullaniyor, bu yuzden "set -u" yok.
HERE=$(cd "$(dirname "$0")" && pwd)
WS=$HERE/../ws
BAGDIR=$HERE/../kayitlar

ROTA=gidis_donus:8; DUNYA=bos; KAYIT=; PROFIL=lidar; HIZ=0.2; GUI=false; DENEME=3
MUTLAK=; SURTUNME=; BASLANGIC=
while [ $# -gt 0 ]; do
  case $1 in
    --rota)   ROTA=$2; shift 2 ;;
    --dunya)  DUNYA=$2; shift 2 ;;
    --kayit)  KAYIT=$2; shift 2 ;;
    --profil) PROFIL=$2; shift 2 ;;
    --hiz)    HIZ=$2; shift 2 ;;
    --deneme) DENEME=$2; shift 2 ;;
    --mutlak) MUTLAK=--mutlak; shift ;;
    --baslangic) BASLANGIC="--baslangic=$2"; shift 2 ;;
    --surtunme) SURTUNME=$2; shift 2 ;;
    --gui)    GUI=true; shift ;;
    *) echo "bilinmeyen secenek: $1"; exit 1 ;;
  esac
done

# Kamera render'i kontrol dongusunu takiyor (uzun cevrim orani %0.04 -> %3.66),
# bu yuzden ihtiyac duyulmadikca kamera hic takilmaz.
case $PROFIL in
  lidar)  KAMERA_KAPAT=true;  LIDAR_KAPAT=false ;;
  kamera) KAMERA_KAPAT=false; LIDAR_KAPAT=true  ;;
  tumu)   KAMERA_KAPAT=false; LIDAR_KAPAT=false ;;
  *) echo "bilinmeyen profil: $PROFIL"; exit 1 ;;
esac

source /opt/ros/jazzy/setup.bash
source $WS/install/setup.bash
export DISPLAY=${DISPLAY:-:0}

# /foot_contacts kayda dahil: bacak odometrisi donunca sebebini ayirt etmek
# icin temas sinyalinin gorulmesi gerekiyor (cok kucuk bir topic).
ORTAK="/clock /imu/data /joint_states /odom/ground_truth /odom/raw /foot_contacts /tf /tf_static"
case $PROFIL in
  lidar)  TOPICS="$ORTAK /velodyne_points/points" ;;
  # /d455/points bilerek disarida: 100+ MB/s tutuyor ve RTAB-Map bulutu
  # kendisi derinlik+RGB'den uretiyor.
  kamera) TOPICS="$ORTAK /d455/image /d455/depth_image /d455/camera_info" ;;
  tumu)   TOPICS="$ORTAK /velodyne_points/points /d455/image /d455/depth_image /d455/camera_info" ;;
esac

trap '$HERE/temizle.sh >/dev/null 2>&1' EXIT

# Devrilmede tekrar: rota_surucu devrilmeyi 2 koduyla bildirir. Devrilen
# kosunun kaydi kullanilamaz (olculdu: robot acilista dusunce 2747 kare
# tamamen hareketsiz goruntu kaydedildi), bu yuzden silinip yenisi kosulur.
for DENE in $(seq 1 $DENEME); do
echo "### deneme $DENE/$DENEME"
$HERE/temizle.sh >/dev/null 2>&1

# world argumani TAM YOL bekliyor; sadece ad verilince gz "Unable to find or
# download file" deyip sessizce oluyor ve spawner'lar 10 dakika bekliyor.
DUNYA_YOL=$DUNYA
[ -f "$DUNYA_YOL" ] || DUNYA_YOL=$WS/src/unitree_go2_ros2/unitree_go2_description/worlds/$DUNYA.sdf
if [ ! -f "$DUNYA_YOL" ]; then
  echo "HATA: dunya bulunamadi: $DUNYA"; exit 1
fi

echo "### simulasyon (dunya=$DUNYA, profil=$PROFIL, gui=$GUI)"
ros2 launch unitree_go2_sim unitree_go2_launch.py \
     world:=$DUNYA_YOL gui:=$GUI rviz:=false \
     disable_camera:=$KAMERA_KAPAT disable_velodyne_lidar:=$LIDAR_KAPAT \
     ${SURTUNME:+foot_friction:=$SURTUNME} \
     > /tmp/champ_sim.log 2>&1 &

# Kontrolcüler aktif olmadan yurumeye kalkmak kosuyu sessizce bozuyor
# (orn. IMU hic yayinlanmiyor), bu yuzden dogrulaniyor.
BEKLENEN="joint_states_controller joint_group_effort_controller"
for i in $(seq 1 180); do
  EKSIK=""
  for c in $BEKLENEN; do
    ros2 control list_controllers 2>/dev/null | grep -q "^$c .*active" || EKSIK="$EKSIK $c"
  done
  [ -z "$EKSIK" ] && break
  sleep 1
done
if [ -n "$EKSIK" ]; then
  echo "HATA: aktif olmayan kontrolcüler:$EKSIK  (acilis yarisi, tekrar deneyin)"
  exit 1
fi
echo "### kontrolcüler aktif"

# Kaydedilecek butun topicler gercekten yayinda mi
for t in $TOPICS; do
  [ "$t" = "/tf_static" ] && continue
  for i in $(seq 1 90); do
    ros2 topic info $t 2>/dev/null | grep -q 'Publisher count: [1-9]' && break
    sleep 1
  done
  ros2 topic info $t 2>/dev/null | grep -q 'Publisher count: [1-9]' \
    || echo "UYARI: $t yayinda degil"
done
sleep 3

if [ -n "$KAYIT" ]; then
  rm -rf $BAGDIR/$KAYIT; mkdir -p $BAGDIR
  echo "### kayit: $BAGDIR/$KAYIT"
  ros2 bag record -o $BAGDIR/$KAYIT --storage mcap --topics $TOPICS \
       > /tmp/champ_bag.log 2>&1 &
  BAG_PID=$!
  for i in $(seq 1 30); do
    grep -q "All requested topics are subscribed" /tmp/champ_bag.log && break
    sleep 1
  done
  if ! grep -q "All requested topics are subscribed" /tmp/champ_bag.log; then
    echo "UYARI: kayit butun topiclere abone olamadi:"
    for t in $TOPICS; do
      grep -q "Subscribed to topic '$t'" /tmp/champ_bag.log || echo "   eksik: $t"
    done
  fi
fi

# Ayak temasi gelmiyorsa bacak odometrisi sessizce donuyor (hizi sifirlaniyor)
# ve kosu kullanilamaz hale geliyor. Sebebi genelde dunya dosyasinda
# gz-sim-contact-system eklentisinin olmamasi; bu yuzden ROTA BASLAMADAN
# once dogrulaniyor.
# Tek ornek guvenilir degil: robot o anda salinimda olabilir ya da echo
# ilk mesaji yakalayamayabilir, bu yuzden birkac kez denenir.
TEMAS=0
for t in 1 2 3 4 5; do
  TEMAS=$(timeout 8 ros2 topic echo /foot_contacts --once 2>/dev/null | grep -c "^- true")
  [ "${TEMAS:-0}" -gt 0 ] && break
  sleep 2
done
if [ "${TEMAS:-0}" -eq 0 ]; then
  echo "HATA: hicbir ayak temas bildirmiyor — bacak odometrisi calismaz."
  echo "      Dunya dosyasinda gz-sim-contact-system eklentisi var mi?"
  echo "      ($DUNYA_YOL)"
  exit 4
fi
echo "### ayak temasi dogrulandi ($TEMAS ayak)"

echo "### rota: $ROTA  (hiz $HIZ m/s)"
RAPOR=${RAPOR:-/tmp/champ_rota.json}
python3 $HERE/rota_surucu.py --rota=$ROTA --hiz=$HIZ $MUTLAK --json=$RAPOR
DURUM=$?

if [ -n "$KAYIT" ]; then
  sleep 2
  kill -INT $BAG_PID 2>/dev/null; sleep 4
  kill -9 $BAG_PID 2>/dev/null
fi

if [ $DURUM -eq 2 ] && [ $DENE -lt $DENEME ]; then
  echo ">>> devrildi, kayit siliniyor ve tekrar deneniyor"
  [ -n "$KAYIT" ] && rm -rf $BAGDIR/$KAYIT
  $HERE/temizle.sh >/dev/null 2>&1
  sleep 3
  continue
fi

if [ -n "$KAYIT" ] && [ -d "$BAGDIR/$KAYIT" ]; then
  # rosbag2 stdin terminal olmadiginda SIGINT'te metadata.yaml yazmiyor
  [ -f $BAGDIR/$KAYIT/metadata.yaml ] || ros2 bag reindex $BAGDIR/$KAYIT -s mcap
  ros2 bag info $BAGDIR/$KAYIT 2>/dev/null | sed -n '2,30p'
fi
exit $DURUM
done
echo ">>> $DENEME denemede gecerli kosu elde edilemedi"
exit 2
