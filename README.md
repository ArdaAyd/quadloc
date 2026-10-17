# quadloc

Unitree Go2 dört ayaklı robotunun Gazebo simülasyonunda yürütülmesi ve
LiDAR/kamera/IMU verisinin kaydedilmesi için çalışma alanı. Kaydedilen
veriye zorlu sensör koşulları (duman, karanlık, IMU gürültüsü) uygulanabilir
ve FAST-LIO2 ile RTAB-Map çalıştırılıp `evo` ile değerlendirilebilir.

## Ortam

Ubuntu 24.04 · ROS 2 Jazzy · Gazebo Harmonic (Sim 8.15)

Paket sürümleri `snapshots.ros.org/jazzy/2026-10-05` deposundan sabitlenir;
böylece herkeste aynı sürümler kurulur.

## Kurulum

### A. Konteyner (önerilen)

```bash
git clone --recurse-submodules https://github.com/ArdaAyd/quadloc.git
cd quadloc
podman build -t quadloc -f docker/Dockerfile .      # docker da olur
```

Çalıştırma — GPU ile:

```bash
podman run --rm -it --device nvidia.com/gpu=all -v $PWD:/ws/quadloc -w /ws/quadloc quadloc
```

Docker kullanıyorsanız `--device nvidia.com/gpu=all` yerine `--gpus all`.

### B. Doğrudan kurulum

```bash
git clone --recurse-submodules https://github.com/ArdaAyd/quadloc.git
cd quadloc
```

ROS deposunu snapshot'a sabitleyin (konteynerle aynı sürümler gelsin diye):

```bash
sudo apt install -y gnupg ca-certificates
gpg --keyserver keyserver.ubuntu.com --recv-keys 4B63CF8FDE49746E98FA01DDAD19BAB3CBF125EA
gpg --export 4B63CF8FDE49746E98FA01DDAD19BAB3CBF125EA | sudo tee /usr/share/keyrings/ros-snapshot-keyring.gpg > /dev/null
echo "deb [signed-by=/usr/share/keyrings/ros-snapshot-keyring.gpg] http://snapshots.ros.org/jazzy/2026-10-05/ubuntu noble main" | sudo tee /etc/apt/sources.list.d/ros2-snapshot.list
sudo apt update
```

Paketleri kurun:

```bash
sudo apt install -y ros-jazzy-desktop ros-jazzy-ros-gz ros-jazzy-gz-ros2-control \
  ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-robot-localization \
  ros-jazzy-rtabmap-ros ros-jazzy-pcl-ros ros-jazzy-xacro ros-jazzy-realsense2-description
sudo apt install -y python3-colcon-common-extensions python3-opencv python3-numpy \
  python3-pil python3-yaml python3-venv
```

Değerlendirme aracı (`evo`) ayrı bir sanal ortama kurulur:

```bash
python3 -m venv .venv-evo
.venv-evo/bin/pip install -r requirements-evo.txt
```

### Derleme (her iki yolda da)

```bash
source /opt/ros/jazzy/setup.bash
python3 tools/doku_uret.py
python3 tools/dokulu_dunya.py --tip koridor
cd ws && colcon build --symlink-install --packages-skip hardware_unitree_sdk2
source install/setup.bash && cd ..
```

> `--packages-up-to` kullanmayın: `unitree_go2_sim`,
> `unitree_go2_description`'ı yalnız çalışma zamanında arar ve o paket
> atlanır.

## Robotu yürütme

```bash
./tools/champ_kosu.sh --rota gidis_donus:6 --mutlak --baslangic 3,0 \
    --dunya koridor_dokulu --profil lidar --hiz 0.2 --gui
```

Ekranda sırasıyla şunlar görünmeli:

```
### kontrolcüler aktif
### ayak temasi dogrulandi (4 ayak)
### rota: gidis_donus:6  (hiz 0.2 m/s)
   hedef 1/2 ulasildi  (t=21 s, uzaklik 0.45 m)
   hedef 2/2 ulasildi  (t=51 s, uzaklik 0.45 m)
>>> yol (tam hiz): gercek 11.54 m | bacak odom 12.13 m | hata +5.1 %
>>> yol (2 Hz)   : gercek 9.00 m | bacak odom 8.63 m | hata -4.0 %
```

`--gui` ile Gazebo penceresi açılır ve robotun koridorda yürüdüğü görülür.
`--gui` yazılmazsa pencere açılmaz ve koşu daha hızlı biter.

Kayıt almak için sona `--kayit kayit_adi` ekleyin; kayıt `kayitlar/`
altına yazılır.

## Dizinler

```
docs/sistem_anlatimi.md   bileşenlerin ne yaptığı ve nasıl denendiği
docker/                   konteyner tanımı
tools/                    koşu, kayıt işleme ve değerlendirme betikleri
ws/src/                   ROS paketleri (ikisi alt modül)
kayitlar/                 rosbag kayıtları (Git'e girmez)
sonuclar/                 SLAM çıktıları ve metrikler (Git'e girmez)
```
