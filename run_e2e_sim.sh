#!/usr/bin/env bash
# Autoware'i AWSIM ile uçtan uca (e2e) modda başlatır (Docker + CycloneDDS).
# Kullanım: ./run_e2e_sim.sh [harita_klasörü]     (varsayılan: Shinjuku)
#   Durdurmak: docker stop autoware_e2e
# Ön koşul: AWSIM çalışıyor olmalı (./run_awsim.sh).
set -e
IMAGE=ghcr.io/autowarefoundation/autoware:universe-cuda
MAP_DIR=${1:-$HOME/autoware_map/shinjuku}
DATA_DIR=${AUTOWARE_DATA:-$HOME/autoware_data}   # ML modelleri; TensorRT .engine dosyaları buraya yazılır
DIR=$(dirname "$(readlink -f "$0")")

docker rm -f autoware_planning_sim autoware_e2e >/dev/null 2>&1 || true   # aynı ROS domain'de iki Autoware olmasın
xhost +local:docker >/dev/null

docker run -d --name autoware_e2e \
  --net host --ipc host --gpus all \
  -e DISPLAY="$DISPLAY" -e QT_X11_NO_MITSHM=1 -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-61}" \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e CYCLONEDDS_URI=file:///autoware_cfg/cyclonedds.xml \
  -v "$DIR/cyclonedds.xml":/autoware_cfg/cyclonedds.xml:ro \
  -v "$DIR/diag_graph":/opt/autoware/share/autoware_launch/config/system/diagnostics:ro \
  -v "$DIR/planning_common/common.param.yaml":/opt/autoware/share/autoware_launch/config/planning/scenario_planning/common/common.param.yaml:ro \
  -v "$DIR/planning_preset/default_preset.yaml":/opt/autoware/share/autoware_launch/config/planning/preset/default_preset.yaml:ro \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$MAP_DIR":/autoware_map:ro \
  -v "$DATA_DIR":/root/autoware_data:rw \
  --entrypoint bash "$IMAGE" -c '
    source /opt/autoware/setup.bash
    exec ros2 launch autoware_launch e2e_simulator.launch.xml \
      map_path:=/autoware_map \
      vehicle_model:=sample_vehicle \
      sensor_model:=awsim_sensor_kit \
      launch_vehicle_interface:=true'

echo "Autoware (e2e) başlatıldı. Log: docker logs -f autoware_e2e"
echo "İlk açılışta TensorRT motorları derlenir (birkaç dakika sürebilir)."
