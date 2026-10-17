#!/bin/bash
# Tek deney kosusu: bassiz simulasyon + yurume + (istege bagli) rosbag kaydi.
#
#   kosu.sh <sure_s> <ly> <dunya> [kayit_adi] [profil]
#
#   ly      : 1.0 -> 0.40 m/s, 0.75 -> 0.30 m/s, 0.5 -> 0.20 m/s
#   dunya   : default | warehouse | baylands
#   profil  : lidar (FAST-LIO2) | kamera (RTAB-Map/ORB-SLAM3) | tumu
#
# Kayit adi verilmezse yalnizca yurutur, kayit yapmaz.
# ROS setup betikleri tanimsiz degisken kullaniyor, bu yuzden "set -u" yok.
SECS=${1:-60}; LY=${2:-1.0}; WORLD=${3:-default}; NAME=${4:-}; PROFILE=${5:-lidar}
# lidar profilinde kameralari hic takma: kamera render'i kontrol dongusunu takiyor
# (uzun cevrim orani %0.04 -> %3.66) ve robotun devrilme ihtimalini artiriyor.
[ "$PROFILE" = "lidar" ] && CAMERAS=false || CAMERAS=true
HERE=$(cd "$(dirname "$0")" && pwd)
WS=$HERE/../ws
BAGDIR=$HERE/../kayitlar

source /opt/ros/jazzy/setup.bash
source $WS/install/setup.bash
export DISPLAY=${DISPLAY:-:0}

# Her algoritmanin gercekten ihtiyac duydugu topicler. /rgbd_d435/points bilerek disarida:
# 109 MB/s tutuyor ve RTAB-Map bulutu kendisi derinlik+RGB'den uretiyor.
ORTAK="/clock /imu_sensor_broadcaster/imu /joint_states /ground_truth/odom /tf /tf_static"
case $PROFILE in
  lidar)  TOPICS="$ORTAK /scan/points" ;;
  kamera) TOPICS="$ORTAK /rgbd_d435/depth_image /rgbd_d435/image /camera/camera_info" ;;
  tumu)   TOPICS="$ORTAK /scan/points /rgbd_d435/depth_image /rgbd_d435/image /camera/camera_info" ;;
  *) echo "bilinmeyen profil: $PROFILE"; exit 1 ;;
esac

cleanup() {
  ps aux | grep -E "gz sim|rviz2|parameter_bridge|image_bridge|gz_quadruped_playground|robot_state_publisher|spawner|ros2 bag record" \
    | grep -v grep | awk '{print $2}' | xargs -r kill -9 2>/dev/null
  sleep 2
}
trap cleanup EXIT
cleanup

echo "### simulasyon basliyor (dunya=$WORLD, bassiz)"
ros2 launch gz_quadruped_playground gazebo.launch.py \
     world:=$WORLD headless:=true rviz:=false cameras:=$CAMERAS foot_friction:=${FRICTION:-0.6} > /tmp/kosu_sim.log 2>&1 &

# Butun kontrolcülerin aktif oldugunu dogrula. imu_sensor_broadcaster spawner'i
# acilis yarisinda olebiliyor; o zaman IMU hic yayinlanmaz ve kosu sessizce bozulur.
BEKLENEN="joint_state_broadcaster imu_sensor_broadcaster unitree_guide_controller"
for i in $(seq 1 300); do
  EKSIK=""
  for c in $BEKLENEN; do
    ros2 control list_controllers 2>/dev/null | grep -q "^$c .*active" || EKSIK="$EKSIK $c"
  done
  [ -z "$EKSIK" ] && break
  sleep 1
done
if [ -n "$EKSIK" ]; then
  echo "HATA: su kontrolcüler aktif degil:$EKSIK"
  echo "      (acilis yarisi; kosuyu tekrar baslatin)"
  exit 1
fi
echo "### kontrolcüler aktif:$BEKLENEN"
for i in $(seq 1 120); do
  ros2 topic info /ground_truth/odom 2>/dev/null | grep -q 'Publisher count: 1' && break; sleep 1
done
sleep 3

if [ -n "$NAME" ]; then
  rm -rf $BAGDIR/$NAME; mkdir -p $BAGDIR
  echo "### kayit: $BAGDIR/$NAME  (profil=$PROFILE)"
  ros2 bag record -o $BAGDIR/$NAME --storage mcap --topics $TOPICS > /tmp/kosu_bag.log 2>&1 &
  # Yurumeye baslamadan once butun topiclere abone olundugunu dogrula; aksi halde
  # gec yayina giren topicler (orn. IMU) kayitta hic yer almiyor.
  for i in $(seq 1 30); do
    grep -q "All requested topics are subscribed" /tmp/kosu_bag.log && break
    sleep 1
  done
  if ! grep -q "All requested topics are subscribed" /tmp/kosu_bag.log; then
    echo "UYARI: kayit butun topiclere abone olamadi:"
    for t in $TOPICS; do grep -q "Subscribed to topic '$t'" /tmp/kosu_bag.log || echo "   eksik: $t"; done
  fi
fi

python3 $HERE/drive_go2.py $SECS $LY --json=${RAPOR:-/tmp/kosu_rapor.json} ${ROTA:+--rota=$ROTA}
KOD=$?

if [ -n "$NAME" ]; then
  ps aux | grep "[r]os2 bag record" | awk '{print $2}' | xargs -r kill -INT 2>/dev/null
  for i in $(seq 1 30); do
    ps aux | grep -q "[r]os2 bag record" || break
    sleep 1
  done
  # rosbag2 bu kurulumda SIGINT uzerine metadata.yaml yazmiyor (stdin terminal degil),
  # bu yuzden metadata'yi mcap dosyasindan yeniden uret. Bag bu adimdan sonra oynatilabilir.
  if [ ! -f $BAGDIR/$NAME/metadata.yaml ]; then
    ros2 bag reindex $BAGDIR/$NAME -s mcap > /tmp/kosu_reindex.log 2>&1
  fi

  echo "### kayit ozeti"
  ros2 bag info $BAGDIR/$NAME 2>/dev/null | head -25
fi

exit ${KOD:-0}
