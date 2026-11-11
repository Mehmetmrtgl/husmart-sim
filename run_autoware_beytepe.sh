#!/usr/bin/env bash
# Autoware'i Beytepe haritası ve Hüsmart sensör kitiyle (AWSIM e2e) başlatır.
# Autoware'in kendisine dokunulmaz: resmi Docker imajı çalışır, bu repodaki ayar dosyaları konteynere salt-okunur bağlanır.
#
# Önce AWSIM çalışıyor olmalı:  ./run_awsim.sh
# Kullanım:                     ./run_autoware_beytepe.sh            (RViz açılır)
# Durdurmak:                    docker rm -f autoware_e2e
# Log:                          docker logs -f autoware_e2e
#
# Ayarlanabilir değişkenler (varsayılanları aşağıda):
#   HARITA        lanelet2 + map_projector_info.yaml klasörü
#   KAROLAR       karolara bölünmüş nokta bulutu klasörü (GitHub Release'ten indirilir)
#   AUTOWARE_DATA Autoware'in yapay zeka modelleri (~/autoware_data; README "Kurulum" bölümü)
#   IMG           Autoware Docker imajı
#   ROS_DOMAIN_ID AWSIM ile aynı olmalı (varsayılan 61)
#   SOLLAMA=1     sollama yöneticisini (overtake/overtake_manager.py) de başlatır
set -e
DIR=$(dirname "$(readlink -f "$0")")
HARITA=${HARITA:-$DIR/harita/beytepe_yeni_v2}
KAROLAR=${KAROLAR:-$DIR/harita/beytepe_yeni_v2_karolar}
AUTOWARE_DATA=${AUTOWARE_DATA:-$HOME/autoware_data}
# Test edilen imaj (universe-cuda etiketi, 2026-04-13 sürümü; autoware_launch 0.50.0). Etiket değişken olduğu için özete sabitlenir.
IMG=${IMG:-ghcr.io/autowarefoundation/autoware@sha256:b7490c70ba67c85a20d94feba68137ba911b5032c621353d50f0c63d3ee63977}
ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-61}
CFG=/opt/autoware/share/autoware_launch/config

for f in "$HARITA/lanelet2_map.osm" "$KAROLAR/pointcloud_map_metadata.yaml"; do
  [ -e "$f" ] || { echo "HATA: $f yok. README 'Harita' bölümüne bakın (nokta bulutu GitHub Release'ten indirilir)."; exit 1; }
done
[ -d "$AUTOWARE_DATA" ] || echo "UYARI: $AUTOWARE_DATA yok; algılama modelleri (centerpoint, yolox) yüklenemez. README 'Kurulum'."
if ! pgrep -f "AWSIM-Beytepe.x86_64" >/dev/null; then echo "UYARI: AWSIM çalışmıyor görünüyor (önce ./run_awsim.sh)."; fi

docker rm -f autoware_e2e >/dev/null 2>&1 || true
docker run -d --name autoware_e2e --net host --ipc host --gpus all \
  -e DISPLAY="${DISPLAY:-:0}" -e QT_X11_NO_MITSHM=1 -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID" \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e CYCLONEDDS_URI=file:///autoware_cfg/cyclonedds.xml \
  -v "$DIR/cyclonedds.xml":/autoware_cfg/cyclonedds.xml:ro \
  -v "$DIR/diag_graph":$CFG/system/diagnostics:ro \
  -v "$DIR/planning_common/common.param.yaml":$CFG/planning/scenario_planning/common/common.param.yaml:ro \
  -v "$DIR/planning_preset/default_preset.yaml":$CFG/planning/preset/default_preset.yaml:ro \
  -v "$DIR/planning_crosswalk/crosswalk.param.yaml":$CFG/planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner/crosswalk.param.yaml:ro \
  -v "$DIR/husmart_sensor_kit":/opt/husmart:ro \
  -v "$DIR/husmart_sensor_kit/override/tier4_localization_component.launch.xml":/opt/autoware/share/autoware_launch/launch/components/tier4_localization_component.launch.xml:ro \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$HARITA":/autoware_map:ro -v "$KAROLAR":/autoware_map_karolar:ro \
  -v "$AUTOWARE_DATA":/root/autoware_data:rw \
  --entrypoint bash "$IMG" -c 'source /opt/autoware/setup.bash; export AMENT_PREFIX_PATH=/opt/husmart:$AMENT_PREFIX_PATH; exec ros2 launch autoware_launch e2e_simulator.launch.xml map_path:=/autoware_map_karolar lanelet2_map_file:=../autoware_map/lanelet2_map.osm pointcloud_map_file:=pointcloud_map vehicle_model:=sample_vehicle sensor_model:=husmart_sensor_kit launch_vehicle_interface:=true' >/dev/null
echo "Autoware başlatıldı (konteyner autoware_e2e). Konum bulma bekleniyor (en fazla 4 dk)..."
if timeout 240 bash -c 'until docker logs autoware_e2e 2>&1 | grep -q "align server succeeded"; do sleep 5; done'; then
  echo "Konum bulundu. RViz'de '2D Goal Pose' ile hedef verin, sonra 'AUTO' düğmesine basın."
else
  echo "UYARI: 4 dakikada konum bulunamadı. 'docker logs autoware_e2e' çıktısına bakın."
fi

if [ "${SOLLAMA:-0}" = 1 ]; then
  docker rm -f autoware_overtake >/dev/null 2>&1 || true
  docker run -d --rm --name autoware_overtake --network host --ipc host -u "$(id -u):$(id -g)" -e HOME=/tmp -e ROS_LOG_DIR=/tmp/roslog \
    -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID" -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e CYCLONEDDS_URI=file:///autoware_cfg/cyclonedds.xml \
    -e PYTHONUNBUFFERED=1 -e PYTHONDONTWRITEBYTECODE=1 -v "$DIR/cyclonedds.xml":/autoware_cfg/cyclonedds.xml:ro \
    -v "$HARITA":/autoware_map:ro -v "$DIR/overtake":/overtake:ro "$IMG" \
    bash -c 'source /opt/autoware/setup.bash && python3 /overtake/overtake_manager.py --ros-args -r __node:=overtake_manager_$(date +%s) -p use_sim_time:=true -p map_path:=/autoware_map/lanelet2_map.osm' >/dev/null
  echo "Sollama yöneticisi başlatıldı (konteyner autoware_overtake)."
fi
