#!/usr/bin/env bash
# AWSIM'deki aracın şu anki yerini yazar (AWSIM açık olmalı; Autoware gerekmez).
#
# Kullanım:
#   ./konum.sh                          # x, y, yön ve aynı yere dönmek için ./isinla.sh komutu
#   ./konum.sh "burada sağa yol eksik"  # ayrıca notunuzla birlikte notlar/konumlar.txt dosyasına ekler
#
# Konum AWSIM'in gürültüsüz GNSS'inden (anten yeri), yön MTi-7 INS'ten alınır. Değerler harita çerçevesinde, metre.
set -e
DIR=$(dirname "$(readlink -f "$0")")
IMG=${IMG:-ghcr.io/autowarefoundation/autoware@sha256:b7490c70ba67c85a20d94feba68137ba911b5032c621353d50f0c63d3ee63977}

OUT=$(docker run --rm --net host -u "$(id -u):$(id -g)" -e HOME=/tmp -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-61}" \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e CYCLONEDDS_URI=file:///c.xml -v "$DIR/cyclonedds.xml":/c.xml:ro \
  --entrypoint bash "$IMG" -c '
source /opt/autoware/setup.bash
python3 - <<EOF
import math, time, rclpy
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped as P
rclpy.init(); n = rclpy.create_node("konum"); got = {}
n.create_subscription(P, "/sensing/gnss/pose_with_covariance", lambda m: got.setdefault("g", m), qos_profile_sensor_data)
n.create_subscription(P, "/sensing/ins/mti7/pose_with_covariance", lambda m: got.setdefault("i", m), qos_profile_sensor_data)
t0 = time.time()
while len(got) < 2 and time.time() - t0 < 10: rclpy.spin_once(n, timeout_sec=0.2)
if "g" not in got: print("HATA"); raise SystemExit
p = got["g"].pose.pose.position; yon = float("nan")
if "i" in got:
    q = got["i"].pose.pose.orientation
    yon = math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))) % 360
print("%.1f %.1f %.1f %.0f" % (p.x, p.y, p.z, yon))
EOF' 2>/dev/null | tail -1)

if [ -z "$OUT" ] || [ "$OUT" = "HATA" ]; then echo "HATA: konum okunamadı. AWSIM açık mı? (./run_awsim.sh)"; exit 1; fi
read -r X Y Z YON <<< "$OUT"
echo "Konum: x $X  y $Y  (yükseklik $Z m)  yön ${YON}°   [0 = doğu, 90 = kuzey]"
echo "Buraya geri dönmek için: ./isinla.sh $X $Y $YON"
if [ $# -gt 0 ]; then
  mkdir -p "$DIR/notlar"
  printf '%s\tx %s\ty %s\työn %s\t%s\n' "$(date '+%Y-%m-%d %H:%M')" "$X" "$Y" "$YON" "$*" >> "$DIR/notlar/konumlar.txt"
  echo "Not eklendi: notlar/konumlar.txt"
fi
