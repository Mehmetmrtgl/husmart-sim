#!/usr/bin/env python3
"""Hüsmart kamera bag'i incelemesi: ZED X stereo zamanlaması, stereo derinliği - JT128 karşılaştırması, MTi-7 durağan gürültü."""
import sys, os, json, math
import numpy as np
import cv2
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs_py import point_cloud2 as pc2

bag, out = sys.argv[1], sys.argv[2]
os.makedirs(out, exist_ok=True)
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id="mcap"), rosbag2_py.ConverterOptions("cdr", "cdr"))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
want = {"/sensing/camera/zedx/left/image_rect_color", "/sensing/camera/zedx/right/image_rect_color",
        "/sensing/camera/zedx/left/camera_info", "/sensing/camera/zedx/right/camera_info",
        "/sensing/lidar/top/pointcloud_raw_ex", "/sensing/lidar/ot128_top/pointcloud_raw_ex", "/sensing/imu/mti7/imu_raw", "/tf_static", "/localization/kinematic_state"}
st = lambda h: h.stamp.sec + h.stamp.nanosec * 1e-9
L, R, info, lidar, imu, tfs, ekf = {}, {}, {}, [], [], [], []
KEEP_IMG = 12          # bellek için: her göz için en fazla bu kadar görüntü sakla (eşleşme için damgalar ayrıca tutulur)
lstamps, rstamps = [], []
while r.has_next():
    topic, data, t = r.read_next()
    if topic not in want: continue
    m = deserialize_message(data, get_message(types[topic]))
    if topic.endswith("image_rect_color"):
        s = st(m.header)
        (lstamps if "/left/" in topic else rstamps).append(s)
    elif topic.endswith("camera_info"):
        info["L" if "/left/" in topic else "R"] = m
    elif topic.endswith("pointcloud_raw_ex"):
        if len(lidar) < 2000:
            a = pc2.read_points(m, field_names=("x", "y", "z", "channel"), skip_nans=True)
            lidar.append((st(m.header), np.stack([np.asarray(a[k], float) for k in ("x", "y", "z")], 1), np.asarray(a["channel"], int)))
    elif topic == "/sensing/imu/mti7/imu_raw":
        imu.append((st(m.header), m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z,
                    m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z,
                    m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w))
    elif topic == "/tf_static":
        tfs.extend(m.transforms)
    elif topic == "/localization/kinematic_state":
        p, q = m.pose.pose.position, m.pose.pose.orientation
        ekf.append((st(m.header), p.x, p.y, p.z, q.x, q.y, q.z, q.w, m.twist.twist.linear.x))

# 2. geçiş: iki gözde de bulunan damgalardan aralıklı 6 çift
sel = sorted(set(np.round(lstamps, 6)) & set(np.round(rstamps, 6)))
sel = set(sel[:: max(1, len(sel) // 6)][:6])
r2 = rosbag2_py.SequentialReader()
r2.open(rosbag2_py.StorageOptions(uri=bag, storage_id="mcap"), rosbag2_py.ConverterOptions("cdr", "cdr"))
r2.set_filter(rosbag2_py.StorageFilter(topics=["/sensing/camera/zedx/left/image_rect_color", "/sensing/camera/zedx/right/image_rect_color"]))
while r2.has_next():
    topic, data, t = r2.read_next()
    m = deserialize_message(data, get_message(types[topic])); s = round(st(m.header), 6)
    if s in sel:
        (L if "/left/" in topic else R)[s] = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width, 3).copy()

res = {}
# --- 1) zamanlama
ls, rs = np.array(lstamps), np.array(rstamps)
common = np.intersect1d(np.round(ls, 6), np.round(rs, 6))
dl = np.diff(ls)
res["zedx"] = dict(sol=len(ls), sag=len(rs), ayni_damgali_cift=int(len(common)),
                   ortalama_hz=round(1 / dl.mean(), 2) if len(dl) else None,
                   aralik_ms_yuzdelik_5_50_95=np.percentile(dl * 1e3, [5, 50, 95]).round(1).tolist() if len(dl) else None)
li = np.array([x[0] for x in lidar]); res["jt128"] = dict(tarama=len(lidar), hz=round(1 / np.diff(li).mean(), 2),
    nokta_ort=int(np.mean([len(x[1]) for x in lidar])), kanal_kullanilan=int(len(np.unique(np.concatenate([x[2] for x in lidar])))))

# --- 2) TF zinciri (bag'deki /tf_static)
def q2R(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
par = {}
for tr in tfs:
    T = np.eye(4); T[:3, :3] = q2R(tr.transform.rotation)
    T[:3, 3] = [tr.transform.translation.x, tr.transform.translation.y, tr.transform.translation.z]
    par[tr.child_frame_id] = (tr.header.frame_id, T)
def to_base(f):
    T = np.eye(4)
    while f != "base_link":
        p, Tp = par[f]; T = Tp @ T; f = p
    return T
T_bl_lidar = to_base("ot128_top" if "ot128_top" in par else "hesai_top");  # 2026-11-11 sonrası ot128_top
T_bl_cam = to_base("zedx_left_camera_optical_frame")
T_cam_lidar = np.linalg.inv(T_bl_cam) @ T_bl_lidar
EK = np.array(ekf) if ekf else None
def T_map_bl(tq):
    """EKF pozu (doğrusal ara değer; yönelim en yakın örnekten)."""
    i = int(np.searchsorted(EK[:, 0], tq)); i = min(max(i, 1), len(EK) - 1)
    a, b = EK[i - 1], EK[i]; w = (tq - a[0]) / (b[0] - a[0]) if b[0] > a[0] else 0.0
    T = np.eye(4); T[:3, 3] = a[1:4] + w * (b[1:4] - a[1:4])
    qq = a if w < 0.5 else b
    x, y, z, ww = qq[4:8]
    T[:3, :3] = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * ww), 2 * (x * z + y * ww)],
                          [2 * (x * y + z * ww), 1 - 2 * (x * x + z * z), 2 * (y * z - x * ww)],
                          [2 * (x * z - y * ww), 2 * (y * z + x * ww), 1 - 2 * (x * x + y * y)]])
    return T, float(a[8] + w * (b[8] - a[8]))
res["tf"] = dict(hesai_top=T_bl_lidar[:3, 3].round(3).tolist(), zedx_left_optical=T_bl_cam[:3, 3].round(3).tolist())

# --- 3) stereo derinliği - lidar
K = np.array(info["L"].k).reshape(3, 3); P_R = np.array(info["R"].p).reshape(3, 4)
fx, cx, cy = K[0, 0], K[0, 2], K[1, 2]; B = -P_R[0, 3] / fx
pairs = [s for s in L if s in R]
sg = cv2.StereoSGBM_create(minDisparity=0, numDisparities=192, blockSize=5, P1=8 * 3 * 25, P2=32 * 3 * 25,
                           uniquenessRatio=10, speckleWindowSize=100, speckleRange=2, mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)
stats = []
for k, s in enumerate(sorted(pairs)[:6]):
    gl = cv2.cvtColor(L[s], cv2.COLOR_BGR2GRAY); gr = cv2.cvtColor(R[s], cv2.COLOR_BGR2GRAY)
    disp = sg.compute(gl, gr).astype(np.float32) / 16.0
    j = int(np.argmin(np.abs(li - s))); pts = lidar[j][1]
    hiz = None
    if EK is not None and len(EK) > 2:
        # araç hareketi telafisi: lidar damgasındaki poz -> görüntü damgasındaki poz
        Tl, _ = T_map_bl(li[j]); Ti, hiz = T_map_bl(s)
        Tcl = np.linalg.inv(T_bl_cam) @ np.linalg.inv(Ti) @ Tl @ T_bl_lidar
    else:
        Tcl = T_cam_lidar
    pc = (Tcl @ np.c_[pts, np.ones(len(pts))].T).T[:, :3]
    pc = pc[pc[:, 2] > 1.0]
    u = fx * pc[:, 0] / pc[:, 2] + cx; v = fx * pc[:, 1] / pc[:, 2] + cy
    ok = (u >= 0) & (u < gl.shape[1] - 1) & (v >= 0) & (v < gl.shape[0] - 1)
    u, v, z = u[ok], v[ok], pc[ok, 2]
    d_exp = fx * B / z
    d_meas = disp[v.astype(int), u.astype(int)]
    val = d_meas > 0.5
    err = (d_meas[val] - d_exp[val])
    stats.append(dict(damga=round(s, 3), hiz_kmh=round(hiz * 3.6, 1) if hiz is not None else None, lidar_dt_ms=round((li[j] - s) * 1e3, 1), lidar_goruntude=int(ok.sum()),
                      gecerli=int(val.sum()), disparite_hata_medyan_px=round(float(np.median(err)), 3) if val.any() else None,
                      mutlak_hata_medyan_px=round(float(np.median(np.abs(err))), 3) if val.any() else None,
                      hata_1px_ici_yuzde=round(100 * float(np.mean(np.abs(err) < 1.0)), 1) if val.any() else None,
                      derinlik_m_yuzdelik_5_50_95=np.percentile(z, [5, 50, 95]).round(1).tolist() if len(z) else None))
    if True:
        vis = L[s].copy()
        zc = np.clip((z - 5) / 45, 0, 1)
        for uu, vv, c in zip(u.astype(int), v.astype(int), zc):
            col = tuple(int(x) for x in cv2.applyColorMap(np.uint8([[255 * (1 - c)]]), cv2.COLORMAP_JET)[0, 0])
            cv2.circle(vis, (uu, vv), 2, col, -1)
        cv2.imwrite(os.path.join(out, f"zedx_sol_jt128_izdusum_{k}.jpg"), vis, [cv2.IMWRITE_JPEG_QUALITY, 88])
        cv2.imwrite(os.path.join(out, f"zedx_sol_sag_{k}.jpg"), np.hstack([L[s], R[s]])[::2, ::2], [cv2.IMWRITE_JPEG_QUALITY, 88])
        dv = np.clip(disp, 0, None); dv = (255 * dv / max(dv.max(), 1)).astype(np.uint8)
        cv2.imwrite(os.path.join(out, f"zedx_disparite_{k}.jpg"), cv2.applyColorMap(dv, cv2.COLORMAP_JET)[::2, ::2])
res["stereo_lidar"] = dict(fx=round(fx, 2), cx=cx, cy=cy, taban_m=round(B, 4), ciftler=stats)

# --- 4) MTi-7 durağan
a = np.array(imu); dt = np.diff(a[:, 0])
res["mti7"] = dict(mesaj=len(a), hz=round(1 / dt.mean(), 2), aralik_ms_min_max=[round(dt.min() * 1e3, 2), round(dt.max() * 1e3, 2)],
                   jiroskop_ort_rad_s=a[:, 1:4].mean(0).round(6).tolist(), jiroskop_std_rad_s=a[:, 1:4].std(0).round(6).tolist(),
                   ivme_ort=a[:, 4:7].mean(0).round(4).tolist(), ivme_std=a[:, 4:7].std(0).round(5).tolist(),
                   ivme_norm_ort=round(float(np.linalg.norm(a[:, 4:7], axis=1).mean()), 4))
print(json.dumps(res, ensure_ascii=False, indent=1))
json.dump(res, open(os.path.join(out, "ozet.json"), "w"), ensure_ascii=False, indent=1)
