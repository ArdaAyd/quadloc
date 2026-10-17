#!/bin/bash
# Kaydedilmis bir bag uzerinde SLAM algoritmasi kosturur ve yorungesini kaydeder.
#
#   tools/slam_kosu.sh fastlio <kaynak_bag> <cikti_adi> [hiz]
#
# Neden bag uzerinde: zorlu kosullarin ucu (duman, IMU gurultusu, karanlik)
# kayit sonrasi uygulaniyor, yani ayni temiz kosudan uretilen bozulmus bag'ler
# karsilastiriliyor. SLAM'i canli simulasyonda kosturmak hem gereksiz hem de
# RTF degisimi uzerinden yeni bir bozucu ekler.
ALGO=${1:?algoritma gerekli (fastlio | rtabmap)}
KAYNAK=${2:?kaynak bag gerekli}
AD=${3:?cikti adi gerekli}
HIZ=${4:-1.0}

HERE=$(cd "$(dirname "$0")" && pwd)
WS=$HERE/../ws
CIKTI=$HERE/../sonuclar/$AD
source /opt/ros/jazzy/setup.bash
source $WS/install/setup.bash

[ -d "$KAYNAK" ] || { echo "HATA: bag yok: $KAYNAK"; exit 1; }
rm -rf "$CIKTI"; mkdir -p "$CIKTI"

temizle() {
  # DIKKAT: desenler KURULU IKILILERIN YOLUNU hedefliyor, cipl ciplak dugum
  # adini degil. "pkill -f rtabmap" bu betigin KENDI komut satirini de
  # esliyor (betik "slam_kosu.sh rtabmap ..." olarak cagriliyor) ve betik
  # kendini olduruyordu — hicbir cikti vermeden, kod 1 ile.
  pkill -f "[f]astlio_mapping" 2>/dev/null
  pkill -f "[o]pt/ros/jazzy/lib/rtabmap" 2>/dev/null
  pkill -f "[r]os2 launch rtabmap_launch" 2>/dev/null
  pkill -f "[r]os2 bag play" 2>/dev/null
  pkill -f "[r]os2 bag record" 2>/dev/null
  sleep 1
}
trap temizle EXIT
temizle

case $ALGO in
  fastlio)
    CIKTI_TOPIC=/Odometry
    ros2 launch fast_lio mapping.launch.py config_file:=go2_velodyne.yaml \
         use_sim_time:=true rviz:=false > $CIKTI/slam.log 2>&1 &
    ;;
  rtabmap)
    # Kaynak bag once tools/kamera_slam_bag.py'den gecirilmis olmali:
    # goruntu cercevesi optik cerceveyle degistirilmeli, /tf ve map->odom
    # cikarilmali (aksi halde TF cakismasi olur, bkz. o aracin aciklamasi).
    CIKTI_TOPIC=/rtabmap_odom
    # approx_sync false: RGB ve derinlik ayni damgayi tasiyor (olculdu, birebir).
    # -d: onceki veritabanini sil, kosular birbirine karismasin.
    ros2 launch rtabmap_launch rtabmap.launch.py \
         rgb_topic:=/d455/image depth_topic:=/d455/depth_image \
         camera_info_topic:=/d455/camera_info \
         frame_id:=base_link odom_topic:=$CIKTI_TOPIC \
         approx_sync:=false use_sim_time:=true \
         imu_topic:=/imu/data wait_imu_to_init:=true \
         rtabmap_viz:=false rviz:=false \
         database_path:=$CIKTI/rtabmap.db \
         args:="-d" > $CIKTI/slam.log 2>&1 &
    ;;
  *) echo "bilinmeyen algoritma: $ALGO"; exit 1 ;;
esac

# Dugum ayaga kalkmadan bag oynatmak ilk taramalarin kaybolmasina yol aciyor.
# Kontrol "ros2 node list" ile yapilmiyor: use_sim_time acikken ve /clock
# henuz yayinlanmazken o komut takiliyor. Cikti topic'inin yayincisi
# beklenerek dogrulaniyor.
for i in $(seq 1 60); do
  ros2 topic info $CIKTI_TOPIC 2>/dev/null | grep -q 'Publisher count: [1-9]' && break
  sleep 1
done
if ! ros2 topic info $CIKTI_TOPIC 2>/dev/null | grep -q 'Publisher count: [1-9]'; then
  echo "HATA: SLAM dugumu $CIKTI_TOPIC yayinlamiyor; $CIKTI/slam.log"
  tail -20 $CIKTI/slam.log; exit 2
fi
echo "### $ALGO calisiyor"

ros2 bag record -o $CIKTI/yorunge --storage mcap \
     --topics $CIKTI_TOPIC /odom/ground_truth > $CIKTI/record.log 2>&1 &
BAG_PID=$!
for i in $(seq 1 20); do
  grep -q "All requested topics are subscribed" $CIKTI/record.log && break
  sleep 1
done

echo "### bag oynatiliyor (hiz $HIZ)"
ros2 bag play "$KAYNAK" --clock 100 --rate $HIZ \
     --qos-profile-overrides-path $HERE/qos/bag_play.yaml > $CIKTI/play.log 2>&1
sleep 3

kill -INT $BAG_PID 2>/dev/null; sleep 4; kill -9 $BAG_PID 2>/dev/null
[ -f $CIKTI/yorunge/metadata.yaml ] || ros2 bag reindex $CIKTI/yorunge -s mcap 2>/dev/null

# TUM bicimine cevir (evo icin)
python3 $HERE/bag_to_tum.py $CIKTI/yorunge $CIKTI_TOPIC $CIKTI/slam_tum.txt
python3 $HERE/bag_to_tum.py $CIKTI/yorunge /odom/ground_truth $CIKTI/referans_tum.txt
echo "### cikti: $CIKTI"
