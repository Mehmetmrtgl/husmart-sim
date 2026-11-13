#!/usr/bin/env bash
# AWSIM'deki aracı haritada bir yere ışınlar (AWSIM açık olmalı; Autoware gerekmez).
#
# Kullanım:
#   ./isinla.sh                       # bilinen yerlerin listesi
#   ./isinla.sh nizamiye              # bilinen bir yere
#   ./isinla.sh -150.2 2640.5         # x y (metre, harita çerçevesi; yön 90° = kuzey)
#   ./isinla.sh -150.2 2640.5 270     # x y yön (derece; 0 = doğu, 90 = kuzey, 180 = batı, 270 = güney)
#
# Yükseklik haritadaki en yakın şerit noktasından alınır (+0.8 m; araç yere düşer).
# x y değerlerini ./konum.sh verir. Yeni bir yer eklemek için aşağıdaki YERLER listesine bir satır yazın.
# Not: Autoware açıkken ışınlarsanız konum bulma (NDT) şaşırabilir; gerekirse RViz'de "2D Pose Estimate" ile düzeltin.
set -e
DIR=$(dirname "$(readlink -f "$0")")
IMG=${IMG:-ghcr.io/autowarefoundation/autoware@sha256:b7490c70ba67c85a20d94feba68137ba911b5032c621353d50f0c63d3ee63977}
HARITA=${HARITA:-$DIR/harita/beytepe_yeni_v2}

#      ad          x        y        yön(°)  açıklama
YERLER="
baslangic   181.45  -192.69   83.7   tur başlangıç noktası
nizamiye   -141.8   2626.0    95.0   nizamiye (ana giriş), kuzeye bakar, dönüşe 40 m
yurt        -24.0   1360.6     5.2   yurt binası önü
"

if [ $# -eq 0 ]; then
  echo "Bilinen yerler:"; echo "$YERLER" | sed '/^$/d' | awk '{printf "  %-11s x %8s  y %8s  yön %6s   ", $1, $2, $3, $4; for (i=5;i<=NF;i++) printf "%s ", $i; print ""}'
  echo "Kullanım: ./isinla.sh <ad>   ya da   ./isinla.sh <x> <y> [yön]"; exit 0
fi
if [ $# -eq 1 ]; then
  satir=$(echo "$YERLER" | awk -v a="$1" '$1==a {print $2, $3, $4}')
  [ -n "$satir" ] || { echo "HATA: '$1' bilinmiyor. Liste için: ./isinla.sh"; exit 1; }
  set -- $satir
fi
X=$1; Y=$2; YON=${3:-90}
for v in "$X" "$Y" "$YON"; do [[ "$v" =~ ^-?[0-9]+(\.[0-9]+)?$ ]] || { echo "HATA: sayı değil: $v"; exit 1; }; done

docker run --rm --net host -u "$(id -u):$(id -g)" -e HOME=/tmp -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-61}" \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e CYCLONEDDS_URI=file:///c.xml -v "$DIR/cyclonedds.xml":/c.xml:ro \
  -v "$HARITA":/harita:ro -e X="$X" -e Y="$Y" -e YON="$YON" --entrypoint bash "$IMG" -c '
source /opt/autoware/setup.bash
python3 - <<EOF
import os, math, time, re
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped as P
x, y, yon = float(os.environ["X"]), float(os.environ["Y"]), float(os.environ["YON"])
# yükseklik: en yakın lanelet düğümü
best = (1e18, 0.0)
txt = open("/harita/lanelet2_map.osm").read()
for m in re.finditer(r"local_x\" v=\"([-\d.]+)\" />\s*<tag k=\"local_y\" v=\"([-\d.]+)\" />\s*<tag k=\"ele\" v=\"([-\d.]+)\"", txt):
    d = (float(m.group(1)) - x) ** 2 + (float(m.group(2)) - y) ** 2
    if d < best[0]: best = (d, float(m.group(3)))
z = best[1] + 0.8
if math.sqrt(best[0]) > 30: print("UYARI: en yakın yol %.0f m uzakta; araç yolun dışına düşebilir." % math.sqrt(best[0]))
rclpy.init(); n = rclpy.create_node("isinla")
pub = n.create_publisher(P, "/awsim/awsim_rviz_plugins/pose_teleport/pose_with_covariance", 1)
msg = P(); msg.header.frame_id = "map"
msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = x, y, z
msg.pose.pose.orientation.z = math.sin(math.radians(yon) / 2); msg.pose.pose.orientation.w = math.cos(math.radians(yon) / 2)
time.sleep(1.5)
for _ in range(3): pub.publish(msg); time.sleep(0.3)
print("Işınlandı: x %.1f  y %.1f  z %.1f  yön %.0f°" % (x, y, z, yon))
EOF'
