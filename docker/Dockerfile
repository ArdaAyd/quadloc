# Simülasyon ortamı — ROS 2 Jazzy + Gazebo Harmonic
#
# SÜRÜM SABİTLEME:
#   1. Taban imaj digest ile sabit
#   2. ROS paketleri snapshots.ros.org'daki tarihli depodan
#   3. evo requirements dosyasından
# Amaç: aynı imaj bugün de altı ay sonra da aynı sürümleri kursun.
#
# Snapshot 2026-10-05 seçildi çünkü host'ta kurulu 604 ros-jazzy paketinin
# TAMAMINI birebir aynı sürümde içeriyor (ölçüldü, fark sıfır — bkz.
# docs/surumler/fark_host_snapshot.txt).
#
# İnşa (podman VE docker ile çalışır):
#   podman build -t quadloc -f docker/Dockerfile .
#   docker build -t quadloc -f docker/Dockerfile .
#
# Çalıştırma — GPU ile:
#   podman:  podman run --rm -it --device nvidia.com/gpu=all \
#                -v $PWD:/ws/quadloc -w /ws/quadloc quadloc
#   docker:  docker run --rm -it --gpus all \
#                -v $PWD:/ws/quadloc -w /ws/quadloc quadloc
#
# GPU olmadan (yazılımsal render, yavaş):
#   -e LIBGL_ALWAYS_SOFTWARE=1

FROM docker.io/osrf/ros@sha256:2f520187e84304fffb60d19c9a7d8e2e79d563a6e579a090427e5994eb1b8c45

ENV DEBIAN_FRONTEND=noninteractive
ARG ROS_SNAPSHOT=2026-10-05

# --- 1. apt kaynağını snapshot'a çevir ---
# packages.ros.org eski sürümleri tutmuyor; sürüm listesiyle sabitleme bir
# süre sonra bozulur. Snapshot bunu çözer.
#
# DİKKAT:
# snapshots.ros.org, packages.ros.org'dan FARKLI bir anahtarla imzalı.
# Taban imajdaki keyring yalnız C1CF6E31...'i içeriyor. Snapshot anahtarı
# (aşağıdaki parmak izi) ne repo.ros2.org/repos.key'de ne de
# rosdistro/ros.asc'de yayınlanmış; anahtar sunucusundaki kimliği
# "ROS Snapshot builder <rosbuild@ros.org>" diyor ama bu kendi beyanıdır.
# Bu yüzden anahtar PARMAK İZİ SABİTLENEREK alınıyor: "sunucu ne verirse"
# değil, tam olarak bu anahtar. Denetlenebilir olsun diye burada yazılı.
# İmza kontrolü KAPATILMADI.
ARG ROS_SNAPSHOT_KEY=4B63CF8FDE49746E98FA01DDAD19BAB3CBF125EA
RUN set -eux; \
    apt-get update && apt-get install -y --no-install-recommends gnupg ca-certificates; \
    gpg --batch --keyserver keyserver.ubuntu.com --recv-keys "${ROS_SNAPSHOT_KEY}"; \
    gpg --batch --list-keys --with-colons "${ROS_SNAPSHOT_KEY}" \
      | grep -q "^fpr:::::::::${ROS_SNAPSHOT_KEY}:"; \
    gpg --batch --export "${ROS_SNAPSHOT_KEY}" > /usr/share/keyrings/ros-snapshot-keyring.gpg; \
    rm -f /etc/apt/sources.list.d/ros2*.list /etc/apt/sources.list.d/ros2*.sources; \
    echo "deb [signed-by=/usr/share/keyrings/ros-snapshot-keyring.gpg] \
http://snapshots.ros.org/jazzy/${ROS_SNAPSHOT}/ubuntu noble main" \
      > /etc/apt/sources.list.d/ros2-snapshot.list; \
    cat /etc/apt/sources.list.d/ros2-snapshot.list; \
    rm -rf /var/lib/apt/lists/*

# --- 2. Taban imajın ÖNCEDEN KURULU paketlerini snapshot'a hizala ---
# Taban imaj ROS paketlerini kurulu getiriyor ve bunlar imajın yapıldığı
# tarihin (≈2026-06) sürümleri. apt kaynağını snapshot'a çevirmek onları
# kendiliğinden değiştirmiyor; ölçüldü: 209 pakette host ile fark kalıyordu.
# Bu yüzden snapshot İÇİNDE bir kez dist-upgrade yapılıyor.
#
# Bu, kaldırılan '--only-upgrade ros-jazzy-*' satırından farklıdır:
# o, packages.ros.org'daki EN SON sürüme çekiyordu (tarihe göre değişir);
# bu ise sabit bir snapshot'a çekiyor, yani deterministik.
RUN apt-get update && apt-get dist-upgrade -y && rm -rf /var/lib/apt/lists/*

# --- 3. Paketler ---
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-ros-gz \
      ros-jazzy-gz-ros2-control \
      ros-jazzy-ros2-control \
      ros-jazzy-ros2-controllers \
      ros-jazzy-robot-localization \
      ros-jazzy-rtabmap-ros \
      ros-jazzy-pcl-ros \
      ros-jazzy-xacro \
      ros-jazzy-realsense2-description \
      python3-colcon-common-extensions \
      python3-opencv python3-numpy python3-pil python3-yaml \
      python3-venv python3-pip \
      git wget \
    && rm -rf /var/lib/apt/lists/*

# --- 4. evo (PEP 668 nedeniyle ayrı venv, requirements ile sabit) ---
# Venv SİSTEM PAKETLERİNİ ALMAZ: host'taki pilot kurulum
# --system-site-packages kullanıyordu ve numpy 1.26.4'e düşüyordu.
# Ölçüldü: numpy 1.26.4 ile 2.5.4 aynı ATE'yi veriyor (0.033168, altı
# basamak), yani sonuç etkilenmiyor; yine de sabit olan tercih edildi.
COPY requirements-evo.txt /tmp/evo_requirements.txt
RUN python3 -m venv /opt/venv-evo \
 && /opt/venv-evo/bin/pip install --no-cache-dir -r /tmp/evo_requirements.txt \
 && /opt/venv-evo/bin/evo_ape --help > /dev/null

# DIKKAT: venv PATH'e EKLENMEZ. Eklenirse 'python3' venv'in python'una
# cozulur ve o venv sistem paketlerini gormez; tools/ betikleri cv2/numpy
# bulamaz (olculdu: doku_uret.py "No module named 'cv2'" verdi, ardindan
# materials/ uretilemedigi icin unitree_go2_description derlenemedi).
# evo her zaman TAM YOLLA cagrilir: /opt/venv-evo/bin/evo_ape

WORKDIR /ws/quadloc

RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc \
 && echo "[ -f /ws/quadloc/ws/install/setup.bash ] && source /ws/quadloc/ws/install/setup.bash" >> /root/.bashrc

CMD ["/bin/bash"]
