#!/usr/bin/env python3
"""Hüsmart tur bag'i incelemesi.
Kullanım: analiz_surus.py <bag> <çıktı_klasörü>
Yer gerçeği: AWSIM'in gürültüsüz GNSS'i (/sensing/gnss/pose_with_covariance, Hüsmart setinde MTi-7 anteni konumunda,
base_link (0.40, 0, 1.82)). base_link gerçeği = anten - R(EKF yönü) * kaldıraç.
"""
import sys, os, json, math, bisect
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs_py import point_cloud2 as pc2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

bag, out = sys.argv[1], sys.argv[2]
os.makedirs(out, exist_ok=True)
ANT = np.array([0.40, 0.0, 1.82])          # base_link -> GNSS anteni
MTI = np.array([1.20, 0.0, 0.45])          # base_link -> MTi-7
LAT0, LON0 = 39.87116440956932, 32.7356890084472

def st(h): return h.stamp.sec + h.stamp.nanosec * 1e-9
def q2R(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
def yaw_of(x, y, z, w): return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

# ileri Transverse Mercator (Snyder), WGS84, k0=1 — NavSatFix'i harita çerçevesine çevirmek için
A, F = 6378137.0, 1 / 298.257223563; E2 = F * (2 - F); EP2 = E2 / (1 - E2)
def M(p): return A * ((1 - E2 / 4 - 3 * E2 ** 2 / 64 - 5 * E2 ** 3 / 256) * p - (3 * E2 / 8 + 3 * E2 ** 2 / 32 + 45 * E2 ** 3 / 1024) * math.sin(2 * p)
                      + (15 * E2 ** 2 / 256 + 45 * E2 ** 3 / 1024) * math.sin(4 * p) - (35 * E2 ** 3 / 3072) * math.sin(6 * p))
def tmerc(lat, lon):
    p, l = math.radians(lat), math.radians(lon); p0, l0 = math.radians(LAT0), math.radians(LON0)
    N = A / math.sqrt(1 - E2 * math.sin(p) ** 2); T = math.tan(p) ** 2; C = EP2 * math.cos(p) ** 2; Aa = (l - l0) * math.cos(p)
    x = N * (Aa + (1 - T + C) * Aa ** 3 / 6 + (5 - 18 * T + T * T + 72 * C - 58 * EP2) * Aa ** 5 / 120)
    y = M(p) - M(p0) + N * math.tan(p) * (Aa * Aa / 2 + (5 - T + 9 * C + 4 * C * C) * Aa ** 4 / 24 + (61 - 58 * T + T * T + 600 * C - 330 * EP2) * Aa ** 6 / 720)
    return x, y

r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id="mcap"), rosbag2_py.ConverterOptions("cdr", "cdr"))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
cls = {k: get_message(v) for k, v in types.items()}
ekf, gt, ndt_tp, ndt_nvtl, ndt_it, ndt_ms = [], [], [], [], [], []
scans, chan_hits, elev_all, rh_all = [], np.zeros(129, int), [], []  # 129: OT128 (AWSIM) halkaları 1..128
objs, mrm, opm, imu, fix, ins = [], [], [], [], [], []
img_stamps = {"L": [], "R": []}
diag_loc = {}
sample_scans = {}
n_scan = 0
SKIP = {"/sensing/camera/zedx/left/image_rect_color", "/sensing/camera/zedx/right/image_rect_color"}
while r.has_next():
    topic, data, t = r.read_next()
    if topic in SKIP:
        # görüntünün yalnız başlığını çöz (tam çözme pahalı): CDR'de header.stamp ilk 8 bayttan sonra (4 bayt kapsülleme)
        sec = int.from_bytes(data[4:8], "little", signed=True); nsec = int.from_bytes(data[8:12], "little")
        img_stamps["L" if "/left/" in topic else "R"].append(sec + nsec * 1e-9)
        continue
    if topic not in cls: continue
    if topic == "/localization/kinematic_state":
        m = deserialize_message(data, cls[topic]); p, q = m.pose.pose.position, m.pose.pose.orientation
        ekf.append((st(m.header), p.x, p.y, p.z, q.x, q.y, q.z, q.w, m.twist.twist.linear.x, m.twist.twist.angular.z))
    elif topic == "/sensing/gnss/pose_with_covariance":
        m = deserialize_message(data, cls[topic]); p = m.pose.pose.position; gt.append((st(m.header), p.x, p.y, p.z))
    elif topic == "/localization/pose_estimator/transform_probability":
        m = deserialize_message(data, cls[topic]); ndt_tp.append((t * 1e-9, m.data))
    elif topic == "/localization/pose_estimator/nearest_voxel_transformation_likelihood":
        m = deserialize_message(data, cls[topic]); ndt_nvtl.append((t * 1e-9, m.data))
    elif topic == "/localization/pose_estimator/iteration_num":
        m = deserialize_message(data, cls[topic]); ndt_it.append((t * 1e-9, m.data))
    elif topic == "/localization/pose_estimator/exe_time_ms":
        m = deserialize_message(data, cls[topic]); ndt_ms.append((t * 1e-9, m.data))
    elif topic in ("/sensing/lidar/ot128_top/pointcloud_raw_ex", "/sensing/lidar/top/pointcloud_raw_ex"):  # 2026-11-11 sonrası ot128_top
        m = deserialize_message(data, cls[topic]); n_scan += 1
        a = pc2.read_points(m, field_names=("x", "y", "z", "channel"), skip_nans=True)
        x, y, z, ch = (np.asarray(a[k], float) for k in ("x", "y", "z", "channel"))
        rh = np.hypot(x, y)
        np.add.at(chan_hits, ch.astype(int).clip(0, 128), 1)
        scans.append((st(m.header), len(x), float(rh.min()) if len(x) else np.nan, int((rh < 10).sum()), int(((rh >= 10) & (rh < 20)).sum()),
                      int(((rh >= 20) & (rh < 30)).sum()), int((rh >= 30).sum())))
        if n_scan % 20 == 0:
            elev_all.append(np.degrees(np.arctan2(z, rh))); rh_all.append(rh)
        if n_scan in (50, 2000, 4000, 6000):
            sample_scans[n_scan] = (x, y, z)
    elif topic == "/perception/object_recognition/objects":
        m = deserialize_message(data, cls[topic])
        objs.append((st(m.header), [(o.kinematics.initial_pose_with_covariance.pose.position.x, o.kinematics.initial_pose_with_covariance.pose.position.y,
                                     o.classification[0].label if o.classification else -1) for o in m.objects]))
    elif topic == "/api/fail_safe/mrm_state":
        m = deserialize_message(data, cls[topic]); mrm.append((t * 1e-9, m.state, m.behavior))
    elif topic == "/api/operation_mode/state":
        m = deserialize_message(data, cls[topic]); opm.append((t * 1e-9, m.mode, m.is_autonomous_mode_available))
    elif topic == "/sensing/imu/mti7/imu_raw":
        m = deserialize_message(data, cls[topic]); o = m.orientation
        imu.append((st(m.header), m.angular_velocity.z, yaw_of(o.x, o.y, o.z, o.w)))
    elif topic == "/sensing/gnss/mti7/nav_sat_fix":
        m = deserialize_message(data, cls[topic]); fix.append((st(m.header), m.latitude, m.longitude, m.altitude))
    elif topic == "/sensing/ins/mti7/pose_with_covariance":
        m = deserialize_message(data, cls[topic]); p = m.pose.pose.position; ins.append((st(m.header), p.x, p.y, p.z))
    elif topic == "/diagnostics":
        m = deserialize_message(data, cls[topic])
        for s in m.status:
            if ("ndt" in s.name or "localization" in s.name) and (s.level[0] if isinstance(s.level, bytes) else s.level) > 0:
                key = (s.name, s.message); diag_loc.setdefault(key, []).append(t * 1e-9)

E = np.array(ekf); G = np.array(gt); S = np.array(scans)
t0 = E[0, 0] if len(E) else 0.0
res = {"sure_s": round(E[-1, 0] - E[0, 0], 1) if len(E) else 0}
# yol uzunluğu ve hız
if len(E):
    d = np.hypot(np.diff(E[:, 1]), np.diff(E[:, 2])); res["yol_m"] = round(float(d.sum()), 1)
    res["hiz_kmh"] = dict(ort=round(float(E[:, 8].mean() * 3.6), 1), max=round(float(E[:, 8].max() * 3.6), 1))

# --- lokalizasyon hatası (EKF - gerçek)
def interp_ekf(tq):
    i = bisect.bisect_left(E[:, 0], tq)
    if i <= 0 or i >= len(E): return None
    a, b = E[i - 1], E[i]; w = (tq - a[0]) / (b[0] - a[0]) if b[0] > a[0] else 0
    return a + w * (b - a)
errs = []
for g in G:
    e = interp_ekf(g[0])
    if e is None: continue
    Rm = q2R(*e[4:8]); p_bl = g[1:4] - Rm @ ANT
    dvec = e[1:3] - p_bl[:2]; yw = yaw_of(*e[4:8])
    lon = dvec[0] * math.cos(yw) + dvec[1] * math.sin(yw); lat = -dvec[0] * math.sin(yw) + dvec[1] * math.cos(yw)
    errs.append((g[0], lon, lat, e[3] - p_bl[2], e[8]))
Er = np.array(errs)
if len(Er):
    h = np.hypot(Er[:, 1], Er[:, 2])
    res["lokalizasyon_hatasi_m"] = dict(ornek=len(Er), yatay_medyan=round(float(np.median(h)), 3), yatay_p95=round(float(np.percentile(h, 95)), 3),
                                       yatay_max=round(float(h.max()), 3), yanal_mutlak_p95=round(float(np.percentile(np.abs(Er[:, 2]), 95)), 3),
                                       boyuna_mutlak_p95=round(float(np.percentile(np.abs(Er[:, 1]), 95)), 3),
                                       dusey_medyan=round(float(np.median(Er[:, 3])), 3), yatay_1m_ustu_sure_s=int((h > 1.0).sum()))
# --- NDT
for name, arr in (("ndt_tp", ndt_tp), ("ndt_nvtl", ndt_nvtl), ("ndt_iterasyon", ndt_it), ("ndt_sure_ms", ndt_ms)):
    a = np.array([v for _, v in arr], float)
    if len(a): res[name] = dict(n=len(a), min=round(float(a.min()), 3), p5=round(float(np.percentile(a, 5)), 3), medyan=round(float(np.median(a)), 3), max=round(float(a.max()), 3))
nv = np.array([v for _, v in ndt_nvtl], float)
if len(nv): res["ndt_nvtl_2.3_alti_yuzde"] = round(100 * float((nv < 2.3).mean()), 2)
# --- JT128
if len(S):
    res["jt128"] = dict(tarama=len(S), nokta_ort=int(S[:, 1].mean()), nokta_p5=int(np.percentile(S[:, 1], 5)), nokta_max=int(S[:, 1].max()),
                        en_yakin_yatay_m_medyan=round(float(np.nanmedian(S[:, 2])), 2),
                        oran_r_lt10=round(float(S[:, 3].sum() / S[:, 1].sum()), 4), oran_10_20=round(float(S[:, 4].sum() / S[:, 1].sum()), 4),
                        oran_20_30=round(float(S[:, 5].sum() / S[:, 1].sum()), 4), oran_ge30=round(float(S[:, 6].sum() / S[:, 1].sum()), 4),
                        kanal_isabet_alan=int((chan_hits > 0).sum()), en_yuksek_isabetli_kanal=int(np.nonzero(chan_hits)[0].max()) if chan_hits.any() else None)
# --- MRM / mod
chg = [(round(tt - t0, 1), int(s), int(b)) for k, (tt, s, b) in enumerate(mrm) if k == 0 or (s, b) != (mrm[k - 1][1], mrm[k - 1][2])]
res["mrm_degisimleri"] = chg[:60]
res["mrm_olay_sayisi"] = sum(1 for _, s, _ in chg if s == 2)
# MRM yerleri
mrm_xy = []
for tt, s, b in chg:
    if s == 2:
        e = interp_ekf(tt + t0)
        if e is not None: mrm_xy.append((round(float(e[1]), 1), round(float(e[2]), 1), round(float(e[8] * 3.6), 1), tt))
res["mrm_yerleri_x_y_kmh_t"] = mrm_xy[:40]
# --- algılama
if objs:
    no = np.array([len(o) for _, o in objs]); res["algilanan_nesne_sayisi"] = dict(ort=round(float(no.mean()), 2), max=int(no.max()))
    nearest = []
    for tt, ol in objs:
        e = interp_ekf(tt)
        if e is None or not ol: continue
        nearest.append(min(math.hypot(ox - e[1], oy - e[2]) for ox, oy, _ in ol))
    if nearest: res["en_yakin_algilanan_nesne_m"] = dict(medyan=round(float(np.median(nearest)), 1), p5=round(float(np.percentile(nearest, 5)), 1), min=round(float(min(nearest)), 1))
# --- MTi-7 tutarlılığı
I = np.array(imu)
if len(I) and len(E):
    yaw_e = np.array([yaw_of(*e[4:8]) for e in E])
    ei = np.searchsorted(E[:, 0], I[:, 0]).clip(1, len(E) - 1)
    dyaw = np.degrees(np.angle(np.exp(1j * (I[:, 2] - yaw_e[ei]))))
    dw = I[:, 1] - E[ei, 9]
    res["mti7"] = dict(imu_hz=round(1 / np.diff(I[:, 0]).mean(), 2), yon_farki_ekf_deg=dict(medyan=round(float(np.median(dyaw)), 3), p95_mutlak=round(float(np.percentile(np.abs(dyaw), 95)), 3)),
                       jiroskop_z_eksi_ekf_yaw_hizi=dict(ort=round(float(dw.mean()), 5), std=round(float(dw.std()), 5)))
Fx = np.array(fix)
if len(Fx) and len(G):
    gi = np.searchsorted(G[:, 0], Fx[:, 0]).clip(1, len(G) - 1)
    pts = np.array([tmerc(la, lo) for _, la, lo, _ in Fx])
    # gerçek anten: en yakın 1 Hz gerçeğine EKF hızıyla taşı (GNSS 4 Hz, gerçek 1 Hz)
    errs_fix = []
    for k, (tf_, la, lo, al) in enumerate(Fx):
        e = interp_ekf(tf_)
        if e is None: continue
        p_ant = e[1:4] + q2R(*e[4:8]) @ ANT       # EKF'den anten (yalnız gürültü dağılımı için)
        errs_fix.append(math.hypot(pts[k, 0] - p_ant[0], pts[k, 1] - p_ant[1]))
    if errs_fix: res["mti7_navsatfix_yatay_hata_m"] = dict(n=len(errs_fix), medyan=round(float(np.median(errs_fix)), 3), p95=round(float(np.percentile(errs_fix, 95)), 3))
# --- görüntü damgaları
ls, rs = np.array(img_stamps["L"]), np.array(img_stamps["R"])
if len(ls):
    res["zedx"] = dict(sol=len(ls), sag=len(rs), ayni_damga=int(len(np.intersect1d(np.round(ls, 6), np.round(rs, 6)))),
                       hz=round(1 / np.diff(ls).mean(), 2), aralik_ms_p50_p95=np.percentile(np.diff(ls) * 1e3, [50, 95]).round(1).tolist())
# --- lokalizasyon teşhisleri
res["lokalizasyon_teshis_uyarilari"] = sorted([(k[0], k[1][:80], len(v)) for k, v in diag_loc.items()], key=lambda x: -x[2])[:12]
json.dump(res, open(os.path.join(out, "ozet.json"), "w"), ensure_ascii=False, indent=1)
print(json.dumps(res, ensure_ascii=False, indent=1))

# ---------------- grafikler
if len(E) and len(S):
    fig, ax = plt.subplots(figsize=(7, 9))
    ei = np.searchsorted(E[:, 0], S[:, 0]).clip(0, len(E) - 1)
    sc = ax.scatter(E[ei, 1], E[ei, 2], c=S[:, 1], s=3, cmap="viridis")
    plt.colorbar(sc, ax=ax, label="JT128 nokta / tarama")
    for x, y, v, tt in mrm_xy: ax.plot(x, y, "rx", ms=9, mew=2)
    ax.set_aspect("equal"); ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]"); ax.set_title("Rota: JT128 nokta sayısı (kırmızı x = MRM)")
    fig.tight_layout(); fig.savefig(os.path.join(out, "rota_jt128_nokta.png"), dpi=110); plt.close(fig)
fig, axs = plt.subplots(4, 1, figsize=(11, 10), sharex=True)
if len(E): axs[0].plot(E[:, 0] - t0, E[:, 8] * 3.6, lw=0.8); axs[0].set_ylabel("hız [km/sa]")
if ndt_tp: a = np.array(ndt_tp); axs[1].plot(a[:, 0] - t0, a[:, 1], lw=0.6, label="TP")
if ndt_nvtl: a = np.array(ndt_nvtl); axs[1].plot(a[:, 0] - t0, a[:, 1], lw=0.6, label="NVTL"); axs[1].axhline(2.3, color="r", ls="--", lw=0.8)
axs[1].legend(); axs[1].set_ylabel("NDT skoru")
if len(Er): axs[2].plot(Er[:, 0] - t0, Er[:, 2], ".", ms=2, label="yanal"); axs[2].plot(Er[:, 0] - t0, Er[:, 1], ".", ms=2, label="boyuna"); axs[2].legend(); axs[2].set_ylabel("lok. hatası [m]")
if len(S): axs[3].plot(S[:, 0] - t0, S[:, 1], lw=0.6); axs[3].set_ylabel("JT128 nokta"); axs[3].set_xlabel("süre [s]")
for tt, s, b in chg:
    if s == 2:
        for a_ in axs: a_.axvline(tt, color="r", lw=0.6, alpha=0.6)
fig.tight_layout(); fig.savefig(os.path.join(out, "zaman_serisi.png"), dpi=100); plt.close(fig)
if elev_all:
    el = np.concatenate(elev_all); rh = np.concatenate(rh_all)
    fig, axs = plt.subplots(1, 3, figsize=(15, 4))
    axs[0].bar(range(129), chan_hits, width=1.0); axs[0].set_xlabel("kanal (çatı lidarı; eski QT128/JT128 0..127, OT128 1..128)"); axs[0].set_ylabel("isabet"); axs[0].set_yscale("log")
    axs[1].hist(el, bins=np.arange(-4, 30, 0.5)); axs[1].set_xlabel("yükseliş [°]"); axs[1].set_title("JT128 isabetleri")
    axs[2].hist(rh, bins=np.arange(0, 61, 1)); axs[2].set_xlabel("yatay mesafe [m]"); axs[2].set_title("kör bölge: yakında nokta yok")
    fig.tight_layout(); fig.savefig(os.path.join(out, "jt128_dagilim.png"), dpi=100); plt.close(fig)
for k, (x, y, z) in sample_scans.items():
    fig, ax = plt.subplots(figsize=(7, 7)); ax.scatter(x, y, c=z, s=1, cmap="jet", vmin=-2, vmax=8)
    for rr in (10, 20, 30): ax.add_patch(plt.Circle((0, 0), rr, fill=False, ls="--", lw=0.6))
    ax.set_xlim(-60, 60); ax.set_ylim(-60, 60); ax.set_aspect("equal"); ax.set_title(f"JT128 tarama {k} (üstten; halkalar 10/20/30 m)")
    fig.tight_layout(); fig.savefig(os.path.join(out, f"jt128_tarama_{k}.png"), dpi=90); plt.close(fig)
print("grafikler:", sorted(f for f in os.listdir(out) if f.endswith(".png")))
