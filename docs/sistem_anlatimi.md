# Sistem Anlatımı

Bu belge sistemin nasıl çalıştığını anlatır. Robotik bilgisi gerektirmez.

**Her bileşende "Kendin dene" bölümü var** — kendi terminalinizde 10–15
dakikada çalıştırabileceğiniz bir alıştırma ve ekranda ne görmeniz gerektiği.

Önce bir kez — **host'ta**:
```bash
cd ~/quadloc
source /opt/ros/jazzy/setup.bash
source ws/install/setup.bash
```

**Konteynerde** (yollar farklı):
```bash
cd /ws/quadloc
source /opt/ros/jazzy/setup.bash
source /ws/quadloc/ws/install/setup.bash
# evo: /opt/venv-evo/bin/evo_ape   (host'ta .venv-evo/bin/evo_ape)
```

Aşağıdaki alıştırmalarda `~/quadloc` yazan yerler konteynerde
`/ws/quadloc`, `.venv-evo/bin` yazan yerler `/opt/venv-evo/bin` olur.

---

## 1. Robot modeli ve CHAMP yürüyüş kontrolcüsü

### Ne işe yarıyor

Robot 12 eklemli: her bacakta 3 motor. Biz robota "0.2 m/s ileri git" diyoruz;
bu 12 motorun her an hangi açıda olması gerektiğine birinin karar vermesi
lazım. **CHAMP** bu işi yapan hazır bir yürüyüş kontrolcüsü.

Sezgisel olarak: CHAMP önce "gövde nereye gitmeli" diye düşünür, sonra
"ayaklar yere nereye basmalı" diye bir desen üretir (tırıs yürüyüşünde
çapraz iki ayak birlikte kalkar, diğer ikisi yerde kalır), sonra "ayağı
oraya koymak için eklemler kaç derece olmalı" sorusunu çözer.

**Bacak kinematiği** bu son adımdır. Formülsüz anlatımı: bacağınızı düşünün.
Omuz, dirsek, bilek açılarını bilirseniz elinizin nerede olduğunu
hesaplayabilirsiniz — buna **ileri kinematik** denir, kolaydır. Tersi daha
zordur: "elim şurada olsun" deyip açıları bulmak — buna **ters kinematik**
denir. Robotun bacağı 3 eklemli olduğu için bu denklemin kapalı bir çözümü
vardır; CHAMP onu kullanır.

### Girdi / çıktı

| yön | ne |
|---|---|
| girdi | `/cmd_vel` (geometry_msgs/Twist) — istenen hız: ileri, yana, dönüş |
| girdi | `/joint_states` — eklemlerin şu anki açıları |
| çıktı | eklem komutları (ros2_control üzerinden Gazebo'ya) |
| çıktı | `/odom/raw` — bacak odometrisi (bkz. bileşen 2) |

### Önemli parametreler

`ws/src/unitree_go2_ros2/unitree_go2_sim/config/gait/gait.yaml`:

| parametre | değer | ne değiştirir |
|---|---|---|
| `max_linear_velocity_x` | 0.3 | İleri hız tavanı (m/s). `/cmd_vel` bunun üstünü isterse kırpılır. |
| `max_linear_velocity_y` | 0.25 | **Yana** yürüme tavanı. Robot holonomik: yana da gidebiliyor. |
| `max_angular_velocity_z` | 0.5 | Dönüş hızı tavanı (rad/s). |
| `stance_duration` | 0.25 | Bir ayağın yerde kalma süresi (s). Yürüyüş ritmini belirler. |
| `nominal_height` | 0.225 | Gövdenin normal yüksekliği (m). |
| `odom_scaler` | 1.0 | Bacak odometrisi çıktısını ölçekler. **Bizde 0.9'dan 1.0'a çekildi.** |

### Bizim değişikliklerimiz

1. `odom_scaler` 0.9 → 1.0. Odometri hesabındaki hatalar düzeltilmeden önce
   eksik çıkan yolu telafi etmek için konmuştu; hesap düzelince bu çarpan
   gerçek bir hatayı maskeler hale geldi.
2. Kontrol döngüsü zamanlayıcısı **duvar saatinden sim saatine** çevrildi.
   Neden önemli: duvar saati zamanlayıcısı, simülasyon yavaşladığında sim
   zamanında daha seyrek atar. Ölçtük: duman açıkken render maliyeti
   yüzünden gerçek zaman çarpanı 0.865'ten 0.830'a düşüyordu ve robot tam o
   oranda yavaş yürüyordu. Yani "dumanın SLAM'e etkisi" ölçümüne, dumanın
   *yürüyüşe* etkisi karışıyordu.

### Doğru çalıştığını nasıl anlarız

Simülasyon koşarken:
```bash
ros2 control list_controllers
```
Beklenen: `joint_states_controller` ve `joint_group_effort_controller`
satırlarının ikisi de `active`.

### Kendin dene (10 dk)

```bash
cd ~/quadloc && ./tools/temizle.sh
./tools/champ_kosu.sh --rota duz:4 --mutlak --dunya koridor_dokulu --profil lidar --hiz 0.2 --gui
```

Ekranda görmeniz gerekenler, sırayla:
- `### kontrolcüler aktif`
- `### ayak temasi dogrulandi (4 ayak)`
- Gazebo penceresi açılır, robot ileri yürür
- Sonunda `>>> yol (2 Hz): gercek X m | bacak odom Y m | hata Z %`

`--gui` yazmazsanız pencere açılmaz (daha hızlı koşar).

---

## 2. Bacak odometrisi ve düzeltilen dört hata

### Ne işe yarıyor

Robotun nerede olduğunu **yalnızca bacaklarına bakarak** tahmin eder. Mantık
basit: ayak yere basılıyken yerde sabittir; o ayağın gövdeye göre geriye
kaydığını görürsek, gövde ileri gitmiş demektir. Bu, tekerlekli robottaki
"tekerlek kaç tur döndü" ölçümünün dört ayaklı karşılığıdır.

Bu bilgi SLAM için bir **ek girdi**; LiDAR ya da kamera bozulduğunda hâlâ
bir tahmin sağlar. Raporumuzun 6. maddesi bunun ne kadar işe yaradığını
ölçmekle ilgili.

### Girdi / çıktı

| yön | ne |
|---|---|
| girdi | `/joint_states` — eklem açıları (ayağın gövdeye göre yeri buradan) |
| girdi | `/foot_contacts` — hangi ayak yerde (champ_msgs/ContactsStamped) |
| girdi | `/imu/data` — gövdenin eğimi (roll/pitch) |
| çıktı | `/odom/raw` (nav_msgs/Odometry) |

### Önemli parametreler

| parametre | nerede | ne değiştirir |
|---|---|---|
| `odom_rate` | launch | Odometrinin örnekleme hızı (Hz). Düşükse temas geçişlerinde hata artar. |
| `min_normal_force` | launch | Temasın **başlaması** için gereken kuvvet (N) |
| `contact_release_force` | launch | Temasın **bitmesi** için altına inilmesi gereken kuvvet |
| `contact_source_topic` | launch | Gazebo sensörü mü, yürüyüş fazı tahmini mi |

### Düzeltilen üç hata (+ bir telafi)

Hepsi `champ/include/champ/odometry/odometry.h` içinde, `getVelocities`
fonksiyonunda. 

**1. Dört ayak yerdeyken hız sıfır sayılıyordu.**
Üst akış "bütün bacaklar yerdeyse hesaplanacak bir şey yok" varsayıyordu.
Ama tırıs yürüyüşünde iki çapraz çiftin değişimi sırasında dört ayak kısa
süre birlikte temas eder ve gövde bu sırada ilerlemeye devam eder. Ölçtük:
örneklerin %36'sı bu durumdaydı.

**2. Basış anındaki ilk örnek sayılıyordu.**
Ayak yere yeni değdiğinde ölçülen fark, ayağın havadayken öne yaptığı
hareketi de içerir. Bu, gövdenin *geriye* gittiği gibi okunur. İlginç
ayrıntı: kodda bunu engellemek için gereken `prev_foot_contacts_` değişkeni
zaten saklanıyordu ama hiç kullanılmıyordu.

**3. Ortalama, sabit 2'ye bölünüyordu — gerçek temas sayısına değil.**
Üst akış `delta_x` ve `delta_y`'nin **ikisini de** 2.0'a bölüyordu
(`x_sum += delta_x / 2.0; y_sum += delta_y / 2.0`). Bu sabit 2, "her an iki
ayak yerde" varsayımıyla alınmış bir ortalamadır — tırıs yürüyüşünün nominal
hali. Ama temas sayısı gerçekte 0 ile 4 arasında değişiyor (hata 1'de
anlatılan dört-ayak anları dahil), dolayısıyla sabit bölen yanlış ölçek
veriyor. Düzeltme: gerçek temas sayısına bölmek.

> Not: `delta_theta` hiç bölünmüyordu — yani açısal hız, ayak sayısı kadar
> katlanıyordu. Aynı düzeltme onu da kapsıyor.

**Ek olarak: gövde yalpası telafisi.** Ayak konumu *gövde* çerçevesinde
ölçülüyor. Gövde yürürken yalpalıyor (ölçtük: roll tepe-tepe 0.24 rad).
~0.3 m bacak boyuyla bu, ayağı gövde çerçevesinde ±4 cm oynatır. Bir adımın
yolu ~15 cm olduğundan, telafi edilmezse bu dönüş gövde hareketi sanılır.
Artık IMU'dan roll/pitch alınıp ayak vektörü önce yataya döndürülüyor.

### Doğru çalıştığını nasıl anlarız

```bash
ros2 topic echo /odom/raw --once | grep -A3 position
```
Robot dururken değerler **çok yavaş** değişmeli; yürürken x artmalı.
Sabit kalıyorsa ayak teması gelmiyordur (bkz. bileşen 9).

### Kendin dene (10 dk)

```bash
cd ~/quadloc && ./tools/temizle.sh
./tools/champ_kosu.sh --rota duz:6 --mutlak --baslangic 2,0 --dunya koridor_dokulu --profil lidar --hiz 0.2
```

Sonda iki satır göreceksiniz:
```
>>> yol (tam hiz): gercek X m | bacak odom Y m | hata +a %
>>> yol (2 Hz)   : gercek Z m | bacak odom W m | hata -b %   (salinim payi %c)
```
**İki satırın işareti farklı olabilir** — bu bir hata değil, metriğin
tanımıyla ilgili (bkz. bileşen 8).

---

## 3. Sensörler ve TF ağacı

### Ne işe yarıyor

Robotta üç sensör var ve her biri **kendi çerçevesinde** ölçüm verir. LiDAR
"3 metre ileride duvar var" der ama bu "LiDAR'ın 3 metre ilerisi"dir. Bu
ölçümleri birleştirmek için her sensörün robota göre nerede durduğunu
bilmek gerekir. **TF** bu bilgiyi tutan sistemdir: çerçeveler arası
dönüşümlerin ağacı.

TF bir **ağaçtır**: her çerçevenin tek bir ebeveyni olmalı. İki düğüm aynı
dönüşümü yayınlarsa ağaç tutarsızlaşır ve hesaplar sessizce bozulur.

### Sensörler

| sensör | topic | hız | not |
|---|---|---|---|
| Velodyne VLP-16 LiDAR | `/velodyne_points/points` | 10 Hz | 16 halka, 360°, ölçülen maks menzil 51.6 m |
| IMU | `/imu/data` | 100 Hz | ivme + açısal hız + yönelim |
| RealSense D455 | `/d455/image`, `/d455/depth_image`, `/d455/camera_info` | 15 Hz | 640×480, derinlik 32FC1 (metre), clip 10 m |
| Ayak temas sensörleri | `/lf_foot_contacts` vb. → `/foot_contacts` | ~100 Hz | Gazebo contact sensor |
| Gerçek konum | `/odom/ground_truth` | 50 Hz | simülatörden, referans |

### TF ağacı (ölçüldü)

```
map → odom → base_footprint → base_link → trunk → {imu_link, d455_*, ...}
                                        → velodyne_base_link → velodyne
```
Tek kök (`map`), hiçbir çerçevenin birden fazla ebeveyni yok.

Ölçülen dönüşümler:
- `base_link → trunk` = birim (sıfır)
- `trunk → imu_link` = birim (yani IMU, base_link ile çakışık)
- `base_link → velodyne` = [0.2, 0, 0.1177] m

### İki tuzak

**Tuzak 1: kamera çerçevesi optik değil.** Gazebo köprüsü görüntülere
`frame_id = d455_camera_link` yazıyor. O çerçevenin z ekseni **yukarı**
bakar. Görsel SLAM kütüphaneleri görüntünün çerçevesini "optik" kabul eder
(optik kuralda z **ileri** bakar). Düzeltilmezse RTAB-Map kamerayı yukarı
bakar sanar ve yörünge tamamen yanlış çıkar — **hata vermeden**.
`tools/kamera_slam_bag.py` bunu `d455_camera_color_optical_frame` ile
değiştiriyor.

**Tuzak 2: `map → odom` iki yayıncı.** Launch bunu statik birim dönüşüm
olarak yayınlıyordu; RTAB-Map da kendisi yayınlar. `publish_map_odom:=false`
argümanı eklendi.

### Doğru çalıştığını nasıl anlarız

```bash
ros2 run tf2_tools view_frames
```
PDF üretir. Ya da canlı:
```bash
ros2 run tf2_ros tf2_echo base_link velodyne
```
Beklenen: `Translation: [0.200, 0.000, 0.118]`

### Kendin dene (15 dk)

Bir terminalde simülasyonu başlatın (bileşen 1'deki komut, `--gui` ile).
Başka bir terminalde:

```bash
cd ~/quadloc && source /opt/ros/jazzy/setup.bash && source ws/install/setup.bash
ros2 topic hz /velodyne_points/points     # ~10 Hz beklenir
ros2 topic hz /imu/data                   # ~100 Hz beklenir (sim saatine göre)
ros2 run tf2_ros tf2_echo base_link velodyne
ros2 topic echo /foot_contacts --once     # dört boolean
```

> **Dikkat:** `ros2 topic hz` **duvar saatine** göre ölçer. Simülasyon
> gerçek zamanın 0.87'si hızında koşuyorsa 10 Hz yerine ~8.7 görürsünüz.
> Bu bir hata değil, ölçüm biriminin sonucudur.

---

## 4. Gerçek konum ve yol takipçisi

### Ne işe yarıyor

**Gerçek konum (ground truth):** simülatör robotun nerede olduğunu zaten
bilir. Bunu `/odom/ground_truth` topic'inden alıyoruz. SLAM'in tahmini ile
bunu karşılaştırarak hata hesaplıyoruz. Gerçek robotta böyle bir şey yoktur
(motion capture gerekir) — simülasyonun en büyük avantajı budur.

**Yol takipçisi (`tools/rota_surucu.py`):** robotu her koşuda **aynı**
yoldan yürütür. Neden önemli: robot serbest bırakılınca her koşuda farklı
bir kavis çiziyor; ölçtük, aynı komutla yol uzunluğu 8.5–9.8 m arasında
değişiyordu. Bu saçılma, ölçmek istediğimiz etkiden büyüktü.

### Girdi / çıktı

| yön | ne |
|---|---|
| girdi | `/odom/ground_truth` — nerede olduğu |
| çıktı | `/cmd_vel` — nereye gitmesi gerektiği |
| çıktı | JSON rapor (yol uzunluğu, hata yüzdesi, süre) |

### Rotalar

| rota | anlamı |
|---|---|
| `duz:12` | 12 m ileri |
| `gidis_donus:8` | 8 m ileri, dön, başa gel (180° dönüş içerir) |
| `kare:4` | 4×4 m kare tur (üç kez 90° dönüş) |
| `zigzag:12:2` | 12 m boyunca 2 m genlikli zikzak |

`--mutlak` verilirse rota noktaları **dünya koordinatında** yorumlanır
(dünya üreteci engelleri rotaya göre yerleştirdiği için bu gerekli).
`--baslangic X,Y` verilirse robot önce oraya sürülür ve **ölçüm oradan
başlar** — ölçülen parçanın her koşuda aynı olması için.

### Önemli parametreler

| parametre | değer | ne değiştirir |
|---|---|---|
| `KP_YAW` | 1.2 | Hedefe yönelme kazancı |
| `KP_YANAL_V` | 0.8 | Yanal sapmanın **yanal hıza** katkısı |
| `MAKS_VY` | 0.15 | Yanal hız tavanı (m/s) |
| `DEVRIK_Z` | 0.15 | Gövde bu yüksekliğin altındaysa "devrildi" |
| `SEYRELT_S` | 0.5 | Seyreltilmiş yol için örnekleme aralığı (s) |

### Bizim değişikliklerimiz

- **Geçiş de varış sayılıyor.** Robot hedefi yandan geçerse bir daha varış
  yarıçapına girmiyordu ve hedefin etrafında daire çizip süreyi tüketiyordu
  (ölçtük: 6 m'lik rotada 23.5 m yol).
- **Yanal kayma yanal hızla düzeltiliyor.** Robot dar koridorda sistematik
  olarak yana kayıyordu (7 m'de 1.7 m). Sadece dönerek düzeltmek yavaş
  kalıyordu; CHAMP holonomik olduğu için doğrudan yana yürütmek çözdü.
- **Devrilme tespiti.** Gövde yüksekliği ve roll/pitch izleniyor; devrilen
  koşu 2 koduyla bildirilip tekrarlanıyor. Öncesinde düşen robot süre
  sınırına kadar bekleniyor ve 1.8 GB kullanılamaz kayıt üretiliyordu.

### Kendin dene (10 dk)

```bash
cd ~/quadloc && ./tools/temizle.sh
./tools/champ_kosu.sh --rota kare:4 --mutlak --dunya oda_dagnik --profil lidar --hiz 0.2 --gui
```
Robot kare bir tur atmalı. Ekranda dört kez
`hedef k/4 ulasildi` görmelisiniz. Gazebo penceresinde robotun köşelerde
döndüğünü izleyin.

---

## 5. Dünya üreteci

### Ne işe yarıyor

Raporun üç ortamını (uzun koridor, dağınık oda, hafif engebeli zemin)
**kodla** üretir. Elle SDF yazmak yerine üreteç kullanmanın sebebi: ortam
parametreleri (koridor boyu, engel sayısı, sürtünme) deney değişkeni haline
geliyor ve herkes aynı dünyayı yeniden üretebiliyor.

### Neden doku üretiyoruz

Mevcut dünyaların hepsi düz gri kutulardan oluşuyordu; görüntüde köşe
bulunamıyordu. Doku eklendikten sonra aynı ölçüm çok daha yüksek çıkıyor.

**Ölçüm tanımı:**
- Dedektör: OpenCV ORB, `nfeatures=2000`
- Köşe sayısı: `orb.detect()` sonucu
- Eşleşme: ardışık iki kare arasında BFMatcher (Hamming) + Lowe oran testi
  (0.75) + temel matris RANSAC'ı (3 px); sayılan, **RANSAC içi** eşleşmeler
- Her iki sayı da **aynı FAST eşiğiyle** ölçülmelidir

Dokulu koridorda, 6 kare çifti:

| FAST eşiği | ORB köşe | eşleşen | oran |
|---|---|---|---|
| 20 (OpenCV varsayılanı) | 1103 | 602 | 0.55 |
| 7 (ORB-SLAM3'ün alt sınırı) | 2000 (tavana dayandı) | 1042 | 0.52 |

> "Aynı panel 1949 eşleşme, farklı paneller 20" sayıları **render edilmiş
> kareler değil, 1024×1024 doku dosyalarının kendisi** üzerinde ölçülmüştü;
> o ölçekte köşe sayısı `nfeatures` tavanına dayanıyor. Render ölçeğiyle
> karşılaştırılamaz.

Doku hazır varlık indirilerek değil **yordamsal olarak** üretiliyor: internet
ve Fuel önbelleği gerekmiyor, ve doku zenginliği kontrollü bir değişken
oluyor.

### Girdi / çıktı

| yön | ne |
|---|---|
| girdi | komut satırı parametreleri |
| çıktı | `ws/src/unitree_go2_ros2/unitree_go2_description/worlds/*.sdf` |
| çıktı | `.../materials/textures/*.png` (doku üreteci) |

### Önemli parametreler

| parametre | varsayılan | ne değiştirir |
|---|---|---|
| `--tip` | koridor | koridor / oda / engebe |
| `--uzunluk`, `--genislik` | tipe göre | ortamın boyutları (m) |
| `--engel` | koridor 0, oda 14 | dokulu kutu sayısı |
| `--serbest-serit` | 2.2 | rota boyunca bırakılan engelsiz şerit (m) |
| `--mu` | 0.8 | zemin sürtünme katsayısı |
| `--seviye` | 2 | engebe şiddeti (0=düz, 1=±1cm, 2=±3cm, 3=±6cm) |

### Üç tasarım kararı ve sebepleri

**1. Duvarlar 1 m'lik panellere bölünüyor.** Gazebo ilkel kutu yüzeyine
dokuyu bir kez gerer; 16 m'lik tek kutuda doku aşırı düşük uzamsal frekansa
iner ve köşe üretmez. Ayrıca her panele farklı doku verilince paneller
birbirinden ayırt edilebiliyor (tekrarlı doku yanlış eşleşme üretir;
ölçtük: aynı panel 1949 eşleşme, farklı paneller 20).

**2. Doku ayrıntısı render ölçeğine göre boyutlandırıldı.** İlk sürümde
ayrıntı ince benekteydi; render'da 1 m'lik panel ~100 piksele indiği için
benek ortalanıp kayboluyordu (142 px'te 106 köşe). Harç ~2 cm, tuğla
25×12 cm, levhalar 5–30 cm yapılınca 248 köşeye çıktı.

**3. Engebe heightmap ile.** 1500+ ayrı kutuyla denendiğinde gerçek zaman
çarpanı 0.07'ye düşüyordu; heightmap tek çarpışma yüzeyi olduğu için
neredeyse bedava.

### Kritik ayrıntı: `gz-sim-contact-system`

Dünya dosyasında bu eklenti **yoksa** ayak temas sensörleri hiç yayın
yapmaz, bütün ayaklar "havada" görünür ve bacak odometrisi donar. Bu bizde
oldu: koridor dünyalarında eklenti eksikti ve `/odom/raw` 180 saniye boyunca
sabit kaldı. Kayıt tamamen normal görünüyordu. Üreteç artık her zaman ekliyor.

### Doğru çalıştığını nasıl anlarız

```bash
grep -l "gz-sim-contact-system" ws/src/unitree_go2_ros2/unitree_go2_description/worlds/*.sdf | wc -l
```
Bütün dünyalarda bulunmalı.

### Kendin dene (10 dk)

```bash
cd ~/quadloc
python3 tools/doku_uret.py
python3 tools/dokulu_dunya.py --tip oda --engel 20
```
Çıktı: `tip oda: 10.0 x 10.0 m, 25 duvar paneli, N engel, ...`

Ürettiğiniz dokuya bakın (`materials/textures/duvar_0.png`) — tuğla deseni,
renkli levhalar ve lekeler görmelisiniz. Sonra dünyayı açın:
```bash
./tools/champ_kosu.sh --rota duz:4 --mutlak --dunya oda_dagnik --profil lidar --hiz 0.2 --gui
```

---

## 6. Zorlu koşul modelleri

Dört koşuldan **üçü** kayıt sonrası uygulanıyor (duman, IMU gürültüsü,
karanlık); biri simülatörde koşulmak zorunda (kaygan zemin).

Kayıt sonrası işlemenin üç sebebi:
1. Seviye **sürekli ve fiziksel** bir büyüklükle veriliyor
2. Render maliyeti olmadığı için gerçek zaman çarpanı değişmiyor
3. Tek temiz kayıttan istenildiği kadar seviye üretiliyor (deterministik)

### 6a. Duman — LiDAR

**Denklem:**
```
alpha = -ln(0.05) / V            (Koschmieder: görüş mesafesinden sönümleme)
P_sert  = i · exp(-2·alpha·R) / R²
P_duman = beta · exp(-2·alpha·r) / r²,   beta = alpha/(4π)
```

**Sade açıklama:** Işın hedefe gidip geri döner, yolda zayıflar — bu
`exp(-2·alpha·R)`. Ayrıca ışın hedefe varmadan bir duman parçacığına
çarpabilir; o zaman sensör noktayı **daha yakında** görür. Zayıflamış dönüş
algılama eşiğinin altına düşerse nokta **tamamen kaybolur**. Yani duman iki
şey yapar: sahte yakın dönüşler üretir ve uzaktaki yapıyı siler.

`V` = görüş mesafesi (m). Küçükse yoğun duman. Kaynak: Hahner vd., ICCV 2021.

### 6b. Duman — kamera

**Denklem:**
```
I(x) = J(x)·t(x) + A·(1 - t(x)),    t(x) = exp(-alpha · d(x))
```

**Sade açıklama:** Her pikselde iki şey karışır: sahneden gelen gerçek ışık
(`J`, mesafeyle zayıflamış) ve dumanın kendi parlaklığı (`A`, "örtü ışığı").
Uzaktaki nesneler daha çok örtülür. `d(x)` piksel derinliği — D455 bunu
zaten veriyor, bu yüzden literatürdeki zor kısım (tek görüntüden derinlik
kestirme) bizde gereksiz.

**Önemli:** `alpha` LiDAR'dakiyle **aynı** formülden gelir. Yani tek bir
"görüş mesafesi" ekseni iki sensörü birden kademelendirir.

**Derinlik kanalı:** geçirgenlik 0.05'in altına düşen pikseller geçersiz
işaretlenir. **Modelimizde** V=10 m'ye kadar derinlik kaybı %0, V=5 m'de
%4.6; yani derinlik RGB'den çok daha dayanıklı.

> Dikkat: bu bir **ölçüm bulgusu değil, modelin sonucudur**. RTAB-Map'in
> dumanda 10 m'ye kadar çalışması, bizim derinlik modelimizin derinliği o
> noktaya kadar bozmamasıyla tutarlıdır — gerçek bir D455'in dumanda nasıl

### 6c. Karanlık

**Denklem (zincir):**
```
ışık = srgb_çöz(piksel/255) · (lux/300)
elektron = ışık · 10000
ölçülen = Poisson(elektron) + N(0, 3)
piksel' = srgb_kodla(ölçülen · kazanç / 10000) · 255,   kazanç = min(300/lux, 32)
```

**Sade açıklama:** Karanlıkta asıl bozulma görüntünün kararması **değil**,
sinyal-gürültü oranının düşmesidir. Az ışık = az foton = istatistiksel
dalgalanma baskın. Üstüne kamera otomatik kazancı parlaklığı geri getirmeye
çalışırken gürültüyü de yükseltir. ORB/FAST köşe bulucularını öldüren budur.
Kazancın 32× tavanı olduğu için çok karanlıkta görüntü hem karanlık hem
gürültülü kalır.

**Modellenmeyen:** pozlama süresi ve **hareket bulanıklığı**. Gerçek bir
kamera karanlıkta pozlamayı uzatır; tırısla koşan robotta bu önemli olabilir.
Bu bir **eksiktir**.

**Derinlik karanlıktan etkilenmiyor** — D455 derinliği aktif IR projektörle
üretir. LiDAR da aktif sensördür, o da etkilenmez.

### 6d. IMU gürültüsü

**Denklem:**
```
ölçülen = (1 + ölçek_hatası)·gerçek + bias(t) + beyaz_gürültü(t)
```

**Sade açıklama:** Üç ayrı bozulma var. **Beyaz gürültü** her örnekte
rastgele sapma — ortalaması sıfır, entegre edilince yavaş büyür. **Bias**
sıfır noktasının kayması — entegre edilince *doğrusal* büyür, asıl tehlike
budur. **Ölçek hatası** ölçümün sabit bir oranla yanlış olması.

Yönelim alanı da bozuluyor: roll/pitch ivmeölçerle düzeltilebildiği için
hatası **sınırlı**; yaw'ın mutlak referansı yok, **sınırsız sürüklenir**.

### 6e. Kaygan zemin

Kayıt sonrası uygulan**a**mayan tek koşul; sürtünme fiziği temas çözümünün
içinde. Seviye `foot_friction` ile veriliyor (ayak–zemin sürtünme katsayısı).

### Kendin dene (15 dk)

```bash
cd ~/quadloc
# Duman modelini tek bir bulut üzerinde gör (simülasyon gerekmez):
python3 tools/duman_modeli.py
```
Çıktı: görüş mesafesi arttıkça kalan nokta oranı ve ortalama menzilin nasıl
değiştiğini gösteren bir tablo.

```bash
# Gerçek bir kayda duman uygula (birkaç dakika sürer):
python3 tools/duman_bag.py kayitlar/koridor_temiz /tmp/duman20 --gorus 20
```
Beklenen: `degismedi %96.5 / duman %2.7 / silindi %0.8` benzeri bir özet.

---

## 7. FAST-LIO2 ve RTAB-Map ayarları

### FAST-LIO2 — ne işe yarıyor

LiDAR + IMU kullanan bir **odometri ve haritalama** yöntemi. Her taramayı
biriktirdiği haritaya oturtarak "ne kadar hareket ettim" sorusunu çözer.
IMU'yu hem tarama içi hareketi düzeltmek hem de iki tarama arası tahmin
yapmak için kullanır. Kamera kullanmaz — karanlıktan hiç etkilenmez.

| yön | ne |
|---|---|
| girdi | `/velodyne_points/points`, `/imu/data` |
| çıktı | `/Odometry` (nav_msgs/Odometry) |

**Bizim ayarlarımız** (`ws/src/FAST_LIO/config/go2_velodyne.yaml`) —
hepsi ölçüldü, varsayılan değil:

| parametre | üst akış | bizde | neden |
|---|---|---|---|
| `lid_topic` | /velodyne_points | /velodyne_points/points | ros_gz köprüsünün adı |
| `scan_line` | 32 | **16** | ölçüldü: 16 halka (VLP-16) |
| `blind` | 2.0 | **0.5** | ölçüldü: 2.0 m noktaların **%20.4'ünü** atıyordu — 4 m genişliğindeki koridorda duvarları siliyor |
| `det_range` | 100 | 60 | ölçülen en uzak dönüş 51.6 m |
| `extrinsic_T` | [0,0,0.28] | **[0.2, 0, 0.1177]** | tf_static'ten ölçüldü |

**Kritik ayrıntı — hareket düzeltmesi kapalı.** Gazebo'nun LiDAR'ı
nokta-başına `time` alanı üretmiyor. O alan olmadan PCL değerleri doldurmuyor,
çöp kalıyor ve FAST-LIO çöp zamanlarla hareket düzeltmesi yapıp ıraksıyor.
Ölçülen:

| bulut | ATE |
|---|---|
| ham (zaman alanı yok) | 372 m |
| `time = 0` (FAST-LIO azimuttan türetiyor) | 1.84 m |
| `time = sabit` (düzeltme yok) | **0.033 m** |

Simüle LiDAR **global deklanşör** — Gazebo 360°'yi tek karede render eder,
tarama içinde düzeltilecek hareket yoktur. `tools/bulut_duzelt.py` sabit
zaman yazar. **Gerçek robota geçilirse bu geri alınmalıdır.**

### RTAB-Map — ne işe yarıyor

RGB-D kamera kullanan bir **görsel SLAM**. Görüntüdeki köşe noktalarını
takip ederek hareket kestirir (görsel odometri) ve daha önce gördüğü yerleri
tanıyınca haritayı düzeltir (çevrim kapatma). LiDAR'dan farkı: pasif sensör,
yani ışığa ve kontrasta bağımlı.

| yön | ne |
|---|---|
| girdi | `/d455/image`, `/d455/depth_image`, `/d455/camera_info`, `/imu/data` |
| çıktı | `/rtabmap_odom` (görsel odometri) |

**Bizim ayarlarımız:**

| parametre | değer | neden |
|---|---|---|
| `frame_id` | base_link | robotun gövde çerçevesi |
| `approx_sync` | false | RGB ve derinlik **birebir aynı** damgayı taşıyor (ölçüldü) |
| `wait_imu_to_init` | **true** | aşağıya bakın |
| `odom_topic` | /rtabmap_odom | kayıttaki `/odom` ile çakışmasın |

**IMU kritik çıktı.** RTAB-Map varsayılanda IMU'yu yerçekimi referansı için
kullanmıyor. Açınca ölçülen:

| yapılandırma | ATE |
|---|---|
| IMU kapalı | 0.280 m |
| IMU açık | **0.058 m** |

4.8 kat fark. Tırısla koşan robotta gövde sürekli yalpaladığı için yerçekimi
referansı belirleyici.

**Ön hazırlık şart.** Kayıt `tools/kamera_slam_bag.py`'den geçirilmeli
(optik çerçeve düzeltmesi + `/tf` ve `map→odom` çıkarma). Ayrıca bag
oynatılırken `/tf_static` QoS'u düzeltilmeli; `ros2 bag play` onu
**volatile** yayınlar, TF dinleyicileri **transient_local** bekler ve
statik dönüşümler hiç ulaşmaz. Bu `tools/qos/bag_play.yaml` ile çözüldü.

### Kendin dene (15 dk)

```bash
cd ~/quadloc
python3 tools/bulut_duzelt.py kayitlar/referans_temiz /tmp/ref_fl
./tools/slam_kosu.sh fastlio /tmp/ref_fl deneme_fl
.venv-evo/bin/evo_ape tum sonuclar/deneme_fl/referans_tum.txt \
    sonuclar/deneme_fl/slam_tum.txt --align --t_max_diff 0.05
```
Beklenen: `rmse` satırında ~0.033 m.

---

## 8. Değerlendirme: evo, ATE, RPE

### Ne işe yarıyor

SLAM'in tahmin ettiği yörüngeyi gerçek yörüngeyle karşılaştırıp **tek bir
sayı** üretir. `evo` bunu yapan standart açık kaynak araçtır.

### ATE — Mutlak Yörünge Hatası

İki yörüngeyi üst üste oturtup (hizalayıp) her an için aradaki mesafeye
bakar, karekök ortalamasını alır. **Global tutarlılığı** ölçer: "robot
sonunda doğru yerde mi?"

Hizalama gerekli çünkü SLAM kendi başlangıç noktasını orijin kabul eder;
mutlak konumu bilmez. `--align` bunu yapar (Umeyama).

### RPE — Bağıl Poz Hatası

Belirli bir mesafedeki (bizde 1 m) hareketin ne kadar doğru kestirildiğine
bakar. **Yerel doğruluğu** ölçer: "bir metre giderken ne kadar hata
yapıyor?"

### İkisi neden birlikte gerekli

Ölçtük: duman seviyeleri arttıkça RPE tekdüze artıyor (0.015 → 0.43 → 0.74
→ 0.94 m) ama **ATE artmıyor** (10 m'de 4.48 m, 5 m'de 1.50 m).

Bunun sebebi "hatalar birbirini götürüyor" **değil**. 1 m'lik adımda RPE'nin
0.94 m olması, tahminin o adımda neredeyse hiç ilerlemediği anlamına gelir —
yani kestirim **çökmüştür**. Çökmüş bir kestirimde ATE, rotanın kendi
boyutu civarında **doyuma ulaşır**: tahmin bir yerde takılı kalır, gerçek
yörünge ondan en fazla rota uzunluğu kadar uzaklaşabilir. Dolayısıyla
"5 m'de ATE daha küçük" cümlesi bir başarı ölçüsü değil, doyum noktasının
nerede olduğuyla ilgilidir.

**Kural:**
- Yerel doğruluk **RPE** ile, global tutarlılık **ATE** ile okunur.
- **Çökmüş bir koşunun ATE'si doğruluk ölçüsü değildir.** Çökme ayrı bir
  sonuç olarak raporlanır (başarı/başarısızlık), ortalamaya karıştırılmaz.

### Bacak odometrisi metriği ayrı

Bu SLAM değil; bacak odometrisinin yolu ile gerçek yolun oranı. **Örnekleme
hızına duyarlı** ve işareti değiştirebiliyor:

| yöntem | hata |
|---|---|
| tam hız (50 Hz) | +%3.5 |
| seyreltme 0.5 s | **−%6.0** |
| seyreltme 2.0 s | −%5.9 |
| alçak geçiren filtre 1.0 s | −%6.0 |

Sebep: 50 Hz'de integre edilen yolun ~%24'ü gövdenin **adım salınımı**,
gerçek ilerleme değil. 0.5 s'ten 2 s'e kadar bütün yöntemler −%6'da
birleşiyor; tam hızdaki +%3.5 kestirici gürültüsünün şişirdiği değer.

### Kendin dene (10 dk)

```bash
cd ~/quadloc
.venv-evo/bin/evo_ape tum sonuclar/temiz_fastlio/referans_tum.txt \
    sonuclar/temiz_fastlio/slam_tum.txt --align --t_max_diff 0.05
.venv-evo/bin/evo_rpe tum sonuclar/temiz_fastlio/referans_tum.txt \
    sonuclar/temiz_fastlio/slam_tum.txt --delta 1 --delta_unit m --t_max_diff 0.05
```
Grafik de isterseniz sona `--plot` ekleyin.

---

## 9. Otomasyon ve koşu öncesi denetim

### Ne işe yarıyor

Bir deney koşusu çok sayıda sürecin doğru sırayla ayağa kalkmasını
gerektiriyor. Herhangi biri sessizce başarısız olursa kayıt **normal
görünür ama kullanılamaz**. Bu bizde iki kez oldu. `champ_kosu.sh` bu
yüzden her koşudan önce ön koşulları **doğruluyor**.

### Akış

```
temizle → simülasyon başlat → kontrolcüleri bekle → topic'leri bekle
→ AYAK TEMASINI DOĞRULA → kayıt başlat → rotayı sür → kaydı kapat → bag reindex
```

### Üç denetim ve her birinin yakaladığı hata

| denetim | yakaladığı |
|---|---|
| Kontrolcüler `active` mi | Spawner açılış yarışında ölürse IMU hiç yayınlanmıyor |
| Bütün topic'lerin yayıncısı var mı | Geç yayına giren topic kayıtta hiç yer almıyor |
| **En az bir ayak temas bildiriyor mu** | Dünyada `contact` eklentisi yoksa bacak odometrisi donuyor |

### Devrilmede tekrar

`rota_surucu.py` devrilmeyi 2 koduyla bildirir; `champ_kosu.sh` kaydı silip
yeniden dener (`--deneme N`).

### Bir tuzak: temizlik deseni kendini öldürmesin

`pkill -f "rtabmap"` çağıran **betiğin kendi komut satırını** eşliyor
(betik `slam_kosu.sh rtabmap ...` diye çağrıldığı için). Betik kendini
öldürüyor, hiçbir çıktı vermeden çıkıyordu. Bu projede bu sınıftan **üç**
hata yaşandı. Kural: temizlik desenleri kurulu ikililerin **yolunu**
hedeflemeli (`[o]pt/ros/jazzy/lib/rtabmap`), çıplak adı değil.

### Doğru çalıştığını nasıl anlarız

Koşu başında şu satırları görmelisiniz:
```
### kontrolcüler aktif
### ayak temasi dogrulandi (4 ayak)
```
İkincisi yoksa koşu 4 koduyla durur ve sebebini yazar.

### Kendin dene (10 dk)

Denetimin gerçekten çalıştığını görmek için **kasıtlı olarak bozun**:

```bash
cd ~/quadloc
# contact eklentisi OLMAYAN bir dünya kopyası üret
sed '/gz-sim-contact-system/d' \
  ws/src/unitree_go2_ros2/unitree_go2_description/worlds/koridor_dokulu.sdf \
  > ws/src/unitree_go2_ros2/unitree_go2_description/worlds/bozuk_test.sdf
./tools/champ_kosu.sh --rota duz:3 --mutlak --dunya bozuk_test --profil lidar --deneme 1
```
Beklenen:
```
HATA: hicbir ayak temas bildirmiyor — bacak odometrisi calismaz.
      Dunya dosyasinda gz-sim-contact-system eklentisi var mi?
```
Sonra temizleyin:
```bash
rm ws/src/unitree_go2_ros2/unitree_go2_description/worlds/bozuk_test.sdf
```

Bu alıştırma, denetim olmasaydı kaydın **sessizce** bozuk çıkacağını
gösteriyor.
