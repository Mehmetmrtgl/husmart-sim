#!/usr/bin/env bash
# Beytepe "Yeni yol" v2 haritasıyla Autoware planning simulator (Docker)
# Kullanım: ./run_planning_sim.sh [harita_klasörü]
#   Durdurmak için: docker stop autoware_planning_sim
# DDS: CycloneDDS (FastDDS ile behavior_planner rota sıfırlamada rmw_wait içinde segfault verdi).
#   Host'tan ros2 ile bağlanmak için: export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=file://$HOME/autoware/cyclonedds.xml
# Ön koşul (her açılışta, sudo): net.core.rmem_max=2147483647 ve `ip link set lo multicast on`
set -e
IMAGE=ghcr.io/autowarefoundation/autoware:universe-cuda
MAP_DIR=${1:-$HOME/autoware_map/beytepe_yeni_v2}
DOMAIN_ID=${ROS_DOMAIN_ID:-61}   # host'taki ROS_DOMAIN_ID (şu an 61); host'tan ros2 komutları aynı domain'de görür

xhost +local:docker >/dev/null
docker rm -f autoware_planning_sim >/dev/null 2>&1 || true

docker run -d --name autoware_planning_sim \
  --net host --ipc host --gpus all \
  -e DISPLAY="$DISPLAY" -e QT_X11_NO_MITSHM=1 -e ROS_DOMAIN_ID="$DOMAIN_ID" \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e CYCLONEDDS_URI=file:///autoware_cfg/cyclonedds.xml \
  -v "$(dirname "$(readlink -f "$0")")/cyclonedds.xml":/autoware_cfg/cyclonedds.xml:ro \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$MAP_DIR":/autoware_map:ro \
  --entrypoint bash "$IMAGE" -c '
    source /opt/autoware/setup.bash
    exec ros2 launch autoware_launch planning_simulator.launch.xml \
      map_path:=/autoware_map \
      vehicle_model:=sample_vehicle \
      sensor_model:=sample_sensor_kit'

echo "Başlatıldı. Log: docker logs -f autoware_planning_sim"
