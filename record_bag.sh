#!/usr/bin/env bash
# AWSIM sensör verilerini ros2 bag (MCAP, zstd sıkıştırma) olarak bu bilgisayara kaydeder.
# Ayrı bir kapta (aynı Autoware imajı) çalışır; dosyalar senin kullanıcınla ~/autoware_bags altına yazılır.
#
# Başlat:  ~/autoware/record_bag.sh [kayıt_adı]
# Durdur:  ~/autoware/record_bag.sh stop        (kaydı düzgün kapatır; kabı zorla silme)
# Oynat:   ros2 bag play ~/autoware_bags/<kayıt_adı>   (Autoware imajında veya ROS 2 Humble kurulu yerde)
#
# Not: 1920x1080 ham kamera görüntüsü dakikada birkaç GB yer tutar.
set -e
IMAGE=ghcr.io/autowarefoundation/autoware:universe-cuda
DIR=$(dirname "$(readlink -f "$0")")
BAGS=$HOME/autoware_bags
NAME=autoware_bag

if [ "$1" = "stop" ]; then
  docker kill -s INT $NAME >/dev/null 2>&1 || { echo "Çalışan kayıt yok."; exit 0; }
  for i in $(seq 60); do docker ps -q -f name=^/$NAME$ | grep -q . || break; sleep 1; done
  echo "Kayıt durduruldu. Klasör: $BAGS"; ls -t "$BAGS" | head -3
  exit 0
fi

BAG=${1:-beytepe_$(date +%Y%m%d_%H%M%S)}
mkdir -p "$BAGS"
TOPICS=(
  /clock /tf /tf_static
  # LiDAR (AWSIM, RGL)
  /sensing/lidar/top/pointcloud_raw_ex /sensing/lidar/top/pointcloud_raw
  # Kamera (trafik ışığı kamerası)
  /sensing/camera/traffic_light/image_raw /sensing/camera/traffic_light/camera_info
  # IMU ve GNSS
  /sensing/imu/tamagawa/imu_raw
  /sensing/gnss/pose /sensing/gnss/pose_with_covariance
  # Araç durumu
  /vehicle/status/velocity_status /vehicle/status/steering_status /vehicle/status/gear_status
  /vehicle/status/control_mode /vehicle/status/turn_indicators_status /vehicle/status/hazard_lights_status
  # Referans: Autoware'in konum tahmini ve rota
  /localization/kinematic_state /planning/mission_planning/route
)
docker rm -f $NAME >/dev/null 2>&1 || true
docker run -d --rm --name $NAME --network host --ipc host -u "$(id -u):$(id -g)" \
  -e HOME=/tmp -e ROS_LOG_DIR=/tmp/roslog \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-61}" -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e CYCLONEDDS_URI=file:///autoware_cfg/cyclonedds.xml \
  -v "$DIR/cyclonedds.xml:/autoware_cfg/cyclonedds.xml:ro" -v "$BAGS:/bags" \
  $IMAGE bash -c "source /opt/autoware/setup.bash && exec ros2 bag record --use-sim-time \
    -s mcap --storage-preset-profile zstd_fast -o /bags/$BAG ${TOPICS[*]}" >/dev/null
echo "Kayıt başladı: $BAGS/$BAG   (durdurmak: ~/autoware/record_bag.sh stop)"
