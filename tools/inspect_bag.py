"""ros2 bag içeriğini inceler: her konunun ilk mesajından alan/çerçeve bilgisi, küçük konulardan istatistik.
Kullanım (Autoware imajında): python3 inspect_bag.py <bag_klasörü>
"""
import sys, math, collections
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

bag = sys.argv[1]


def open_reader(topics=None):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id="mcap"),
           rosbag2_py.ConverterOptions("cdr", "cdr"))
    if topics:
        r.set_filter(rosbag2_py.StorageFilter(topics=topics))
    return r


r = open_reader()
types = {t.name: t.type for t in r.get_all_topics_and_types()}
first = {}
# 1) her konunun ilk mesajı (tf_static'in iki mesajı da)
tf_static = []
while r.has_next() and (len(first) < len(types) or len(tf_static) < 2):
    topic, data, t = r.read_next()
    if topic == "/tf_static" and len(tf_static) < 2:
        tf_static.append(deserialize_message(data, get_message(types[topic])))
    if topic not in first:
        first[topic] = deserialize_message(data, get_message(types[topic]))


def stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


print("=== SENSÖRLER")
pc = first["/sensing/lidar/top/pointcloud_raw_ex"]
print("LiDAR pointcloud_raw_ex: frame=%s  nokta=%d  alanlar=%s  point_step=%d" % (
    pc.header.frame_id, pc.width * pc.height, [(f.name, f.datatype) for f in pc.fields], pc.point_step))
pc2 = first["/sensing/lidar/top/pointcloud_raw"]
print("LiDAR pointcloud_raw   : frame=%s  nokta=%d  alanlar=%s" % (
    pc2.header.frame_id, pc2.width * pc2.height, [f.name for f in pc2.fields]))
# nokta istatistiği (ilk tarama)
dt = np.dtype({"names": [f.name for f in pc.fields], "formats": [{7: "<f4", 8: "<f8", 2: "u1", 4: "u2", 6: "u4", 5: "<i4"}[f.datatype] for f in pc.fields],
               "offsets": [f.offset for f in pc.fields], "itemsize": pc.point_step})
a = np.frombuffer(bytes(pc.data), dt)
d = np.sqrt(a["x"] ** 2 + a["y"] ** 2 + a["z"] ** 2)
print("   menzil min/medyan/max %.1f / %.1f / %.1f m   ring sayısı %s   intensity %s..%s" % (
    d.min(), np.median(d), d.max(), len(np.unique(a["channel"])) if "channel" in a.dtype.names else (len(np.unique(a["ring"])) if "ring" in a.dtype.names else "-"),
    a["intensity"].min() if "intensity" in a.dtype.names else "-", a["intensity"].max() if "intensity" in a.dtype.names else "-"))
im = first["/sensing/camera/traffic_light/image_raw"]; ci = first["/sensing/camera/traffic_light/camera_info"]
print("Kamera image_raw: frame=%s  %dx%d  encoding=%s  boyut=%.1f MB/kare" % (im.header.frame_id, im.width, im.height, im.encoding, len(im.data) / 1e6))
print("   camera_info: model=%s  fx=%.1f fy=%.1f cx=%.1f cy=%.1f  bozulma=%s" % (ci.distortion_model, ci.k[0], ci.k[4], ci.k[2], ci.k[5], list(ci.d)))
imu = first["/sensing/imu/tamagawa/imu_raw"]
print("IMU imu_raw: frame=%s  ivme=(%.2f, %.2f, %.2f)  açısal hız=(%.3f, %.3f, %.3f)  yönelim verisi=%s" % (
    imu.header.frame_id, imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z,
    imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z,
    "yok" if imu.orientation_covariance[0] == -1 or (imu.orientation.w == 0 and imu.orientation.x == 0) else "var"))
g = first["/sensing/gnss/pose"]; gc = first["/sensing/gnss/pose_with_covariance"]
print("GNSS pose: frame=%s  konum=(%.2f, %.2f, %.2f)   pose_with_covariance frame=%s  σx=%.2f m" % (
    g.header.frame_id, g.pose.position.x, g.pose.position.y, g.pose.position.z, gc.header.frame_id, math.sqrt(max(gc.pose.covariance[0], 0))))

print("\n=== ARAÇ DURUMU (ilk mesaj)")
for tp in ("/vehicle/status/velocity_status", "/vehicle/status/steering_status", "/vehicle/status/gear_status",
           "/vehicle/status/control_mode", "/vehicle/status/turn_indicators_status", "/vehicle/status/hazard_lights_status"):
    m = first[tp]
    fields = {k: getattr(m, k) for k in m.get_fields_and_field_types() if k not in ("stamp", "header")}
    print("%-42s %s" % (tp, fields))

print("\n=== TF ÇERÇEVELERİ")
for ts in tf_static:
    for tr in ts.transforms:
        t = tr.transform.translation
        print("  statik %-28s -> %-34s (%.2f, %.2f, %.2f)" % (tr.header.frame_id, tr.child_frame_id, t.x, t.y, t.z))
dyn = set()
r = open_reader(["/tf"]); n = 0
while r.has_next() and n < 400:
    _, data, _ = r.read_next(); n += 1
    for tr in deserialize_message(data, get_message("tf2_msgs/msg/TFMessage")).transforms:
        dyn.add((tr.header.frame_id, tr.child_frame_id))
print("  dinamik:", sorted(dyn))

print("\n=== ROTA")
rt = first["/planning/mission_planning/route"]
print("  başlangıç (%.1f, %.1f)  hedef (%.1f, %.1f)  bölüm sayısı %d" % (
    rt.start_pose.position.x, rt.start_pose.position.y, rt.goal_pose.position.x, rt.goal_pose.position.y, len(rt.segments)))

print("\n=== SÜRÜŞ İSTATİSTİĞİ (Autoware konum tahmini + araç hızı)")
r = open_reader(["/localization/kinematic_state", "/vehicle/status/velocity_status", "/vehicle/status/steering_status", "/sensing/gnss/pose"])
xs, ys, zs, ts, vs, st, gx, gy, gt = [], [], [], [], [], [], [], [], []
while r.has_next():
    tp, data, _ = r.read_next()
    m = deserialize_message(data, get_message(types[tp]))
    if tp == "/localization/kinematic_state":
        p = m.pose.pose.position; xs.append(p.x); ys.append(p.y); zs.append(p.z); ts.append(stamp(m.header))
    elif tp == "/vehicle/status/velocity_status":
        vs.append(m.longitudinal_velocity)
    elif tp == "/vehicle/status/steering_status":
        st.append(m.steering_tire_angle)
    else:
        gx.append(m.pose.position.x); gy.append(m.pose.position.y); gt.append(stamp(m.header))
xs, ys, zs, ts, vs, st = map(np.array, (xs, ys, zs, ts, vs, st))
dist = np.hypot(np.diff(xs), np.diff(ys)).sum()
print("  süre %.0f s, katedilen yol %.0f m, hız ort %.1f / en fazla %.1f km/sa" % (ts[-1] - ts[0], dist, vs[vs > 0.5].mean() * 3.6, vs.max() * 3.6))
print("  konum aralığı x %.0f..%.0f  y %.0f..%.0f  yükseklik %.1f..%.1f m (fark %.1f m)" % (xs.min(), xs.max(), ys.min(), ys.max(), zs.min(), zs.max(), zs.max() - zs.min()))
print("  direksiyon açısı en fazla ±%.1f°" % (np.degrees(np.abs(st).max())))
gx, gy, gt = map(np.array, (gx, gy, gt))
idx = np.clip(np.searchsorted(ts, gt), 0, len(ts) - 1)
err = np.hypot(gx - xs[idx], gy - ys[idx])
print("  GNSS ile Autoware konumu farkı: medyan %.2f m, en fazla %.2f m" % (np.median(err), err.max()))
