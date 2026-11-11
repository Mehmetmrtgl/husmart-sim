"""Beytepe v2 — planning simulator sürüş testleri (konteyner içinde çalışır).

Her test: stop → rota temizle → lokalizasyonu başlangıç pozuna ayarla → rota ver →
otonoma al → varış / takılma / zaman aşımına kadar izle → metrikleri yaz.
Kullanım: python3 run_tests.py specs.json /tmp/beytepe_results
"""
import json, math, os, sys, threading, time
import numpy as np
import lanelet2
from lanelet2.projection import UtmProjector

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import AccelWithCovarianceStamped
from autoware_adapi_v1_msgs.srv import (InitializeLocalization, SetRoutePoints, ClearRoute,
                                        ChangeOperationMode)
from autoware_adapi_v1_msgs.msg import (RouteState, OperationModeState, MotionState, MrmState,
                                        LocalizationInitializationState)
from autoware_planning_msgs.msg import LaneletRoute
from autoware_perception_msgs.msg import PredictedObjects

MAP = "/autoware_map/lanelet2_map.osm"
STUCK_S = 30.0          # hareket başladıktan sonra bu kadar süre durursa "takıldı"
GOAL_TOL = 1.5          # m


def aget(a, k, d=""):
    return a[k] if k in a else d


# ---------------------------------------------------------------- harita
def _first_latlon(path):
    """projektör başlangıcı: dosyadaki ilk düğümün lat/lon'u (x,y zaten local_x/local_y'den alınır;
    yalnız UTM bölgesinin doğru olması gerekir — Beytepe ve Shinjuku farklı bölgelerde)"""
    import re
    with open(path) as f:
        for line in f:
            mm = re.search(r'lat="([-0-9.]+)"\s+lon="([-0-9.]+)"', line)
            if mm:
                return float(mm.group(1)), float(mm.group(2))
    return 0.0, 0.0


proj = UtmProjector(lanelet2.io.Origin(*_first_latlon(MAP)))
MAPOBJ, _load_errs = lanelet2.io.loadRobust(MAP, proj)
assert len(MAPOBJ.pointLayer) > 0, f"harita yüklenemedi: {_load_errs[:3]}"
for p in MAPOBJ.pointLayer:
    a = p.attributes
    if "local_x" in a:
        p.x, p.y = float(a["local_x"]), float(a["local_y"])  # z: lanelet2 ele etiketinden zaten okur
RULES = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                      lanelet2.traffic_rules.Participants.Vehicle)
GRAPH = lanelet2.routing.RoutingGraph(MAPOBJ, RULES)
CL = {ll.id: np.array([[q.x, q.y, q.z] for q in ll.centerline]) for ll in MAPOBJ.laneletLayer}


def ll_of(i):
    return MAPOBJ.laneletLayer[i]


def cum(c):
    return np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(c[:, :2], axis=0), axis=1))])


def pose_at(lid, s):
    """lanelet merkez çizgisinde s metredeki konum + yön"""
    c = CL[lid]; d = cum(c); s = float(np.clip(s, 0, d[-1]))
    k = int(np.clip(np.searchsorted(d, s) - 1, 0, len(c) - 2))
    t = (s - d[k]) / max(d[k + 1] - d[k], 1e-9)
    p = c[k] + t * (c[k + 1] - c[k])
    yaw = math.atan2(c[k + 1][1] - c[k][1], c[k + 1][0] - c[k][0])
    return p, yaw


def length(lid):
    return cum(CL[lid])[-1]


def pick_next(lid):
    f = GRAPH.following(ll_of(lid))
    if not f:
        return None
    f.sort(key=lambda x: (aget(x.attributes, "turn_direction") not in ("", "straight"), x.id))
    return f[0].id


def walk_ahead(lid, s, dist):
    """lid üzerinde s'den başlayıp ardıllarla dist metre ilerle"""
    rem = dist + s
    while rem > length(lid):
        rem -= length(lid)
        n = pick_next(lid)
        if n is None:
            return lid, length(lid)
        lid = n
    return lid, rem


def walk_back(lid, dist):
    """lid başlangıcından öncüllerle dist metre geri git"""
    rem = dist
    while True:
        prev = GRAPH.previous(ll_of(lid))
        if not prev:
            return lid, 0.0
        prev.sort(key=lambda x: (aget(x.attributes, "turn_direction") not in ("", "straight"), x.id))
        lid = prev[0].id
        if rem <= length(lid):
            return lid, length(lid) - rem
        rem -= length(lid)


def nearest_lanelet(x, y, yaw):
    """araç konumuna en yakın, yönü uyumlu (<45°) yol lanelet'i ve üzerindeki s"""
    best = (1e9, None, 0.0)
    for ll in MAPOBJ.laneletLayer:
        if aget(ll.attributes, "subtype") != "road":
            continue
        c = CL[ll.id]; a, b = c[:-1, :2], c[1:, :2]; ab = b - a
        t = np.clip(((np.array([x, y]) - a) * ab).sum(1) / np.maximum((ab * ab).sum(1), 1e-12), 0, 1)
        q = a + t[:, None] * ab; d = np.linalg.norm(q - [x, y], axis=1); k = int(d.argmin())
        hd = math.atan2(ab[k, 1], ab[k, 0])
        if abs((hd - yaw + math.pi) % (2 * math.pi) - math.pi) > math.radians(45):
            continue
        if d[k] < best[0]:
            s = cum(c)[k] + t[k] * np.linalg.norm(ab[k])
            best = (d[k], ll.id, float(s))
    return best[1], best[2]


def resolve(spec, node=None):
    st, gl = spec["start"], spec["goal"]
    if st.get("current"):
        p = node.odom.pose.pose; q = p.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        s_lid, s_s = nearest_lanelet(p.position.x, p.position.y, yaw)
    elif "back" in st:
        s_lid, s_s = walk_back(st["lanelet"], st["back"])
    else:
        s_lid, s_s = st["lanelet"], st.get("frac", 0.2) * length(st["lanelet"])
    if "ahead" in gl:
        base = gl.get("after", s_lid)
        g_lid, g_s = walk_ahead(base, length(base) if "after" in gl else s_s, gl["ahead"])
    else:
        g_lid, g_s = gl["lanelet"], gl.get("frac", 0.5) * length(gl["lanelet"])
    return (s_lid, s_s), (g_lid, g_s)


def mk_pose(p, yaw):
    ps = Pose()
    ps.position.x, ps.position.y, ps.position.z = map(float, p)
    ps.orientation.z, ps.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return ps


def dist_to_polylines(xy, polys):
    best = 1e9
    for c in polys:
        a, b = c[:-1, :2], c[1:, :2]
        ab = b - a; t = np.clip(((xy - a) * ab).sum(1) / np.maximum((ab * ab).sum(1), 1e-12), 0, 1)
        d = np.linalg.norm(a + t[:, None] * ab - xy, axis=1).min()
        best = min(best, d)
    return best


def signed_dev(xy, polys):
    """en yakın merkez çizgisine işaretli yanal sapma (+ sol)"""
    best, sgn = 1e9, 1.0
    for c in polys:
        a, b = c[:-1, :2], c[1:, :2]
        ab = b - a; t = np.clip(((xy - a) * ab).sum(1) / np.maximum((ab * ab).sum(1), 1e-12), 0, 1)
        q = a + t[:, None] * ab; d = np.linalg.norm(q - xy, axis=1); k = int(d.argmin())
        if d[k] < best:
            cr = ab[k, 0] * (xy[1] - a[k, 1]) - ab[k, 1] * (xy[0] - a[k, 0])
            best, sgn = d[k], (1.0 if cr >= 0 else -1.0)
    return sgn * best


LC_JUMP = 1.5      # m: şerit değiştirmede, iki şerit merkezinin ortasında işaretli sapmanın en az bu kadar sıçraması
LC_MARGIN_S = 2.0  # s: Autoware şerit değiştirme modülünün bu kadar önce/sonra çalışmış olması yeter


def lane_change_runs(t, dev, lim, lc=None):
    """|sapma| > lim olan ardışık bölümleri bulur ve hangilerinin bilinçli şerit değiştirme olduğunu işaretler.

    Şerit değiştirmede araç bir şerit merkezinden komşu şeridin merkezine geçer: iki merkezin tam ortasında
    "en yakın merkez çizgisi" değişir, işaretli sapma yaklaşık bir şerit genişliği sıçrar (3.5 m şeritte
    -1.75 -> +1.75) ve araç yeni şeridin merkezinde (|sapma| <= lim) bölümü bitirir.
    Ölçüt (bölüm iki uçta da sınır içinde başlayıp bitmeli, kayıt ortasında kesilmemeli):
      * lc (Autoware şerit değiştirme modülü YÜRÜTÜLÜYOR/İPTAL bayrağı) kaydedilmişse: bölümün ±LC_MARGIN_S s
        çevresinde modül çalışmış olmalı. Böylece yarıda bırakılıp eski şeride dönülen (iptal edilen) şerit
        değiştirmeler de ayrılır.
      * modül bilgisi hiç yoksa (eski kayıtlar) yalnız geometri: bölüm içinde işaret değişimiyle en az LC_JUMP m'lik
        sıçrama (araç iki şerit merkezinin ortasından geçti, yani şerit değiştirme tamamlandı).
    Modül çalışmıyorken merkez çizgisinden uzaklaşıp aynı şeride dönen sapmalar (viraj kesme, kayma) sapma sayılır."""
    n = len(dev); runs = []; k = 0
    use_lc = lc is not None and len(lc) == n and bool(np.any(lc))
    while k < n:
        if abs(dev[k]) <= lim:
            k += 1; continue
        a = k
        while k < n and abs(dev[k]) > lim:
            k += 1
        b = k - 1
        seg = dev[a:b + 1]
        bounded = a > 0 and b < n - 1
        crossed = len(seg) > 1 and bool(np.any((np.sign(seg[:-1]) != np.sign(seg[1:])) &
                                               (np.abs(np.diff(seg)) > LC_JUMP)))
        if use_lc:
            w = (t >= t[a] - LC_MARGIN_S) & (t <= t[b] + LC_MARGIN_S)
            intent = bool(np.any(lc[w]))
            is_lc = bounded and intent
        else:
            intent = None
            is_lc = bounded and crossed
        runs.append(dict(a=a, b=b, t0=round(float(t[a]), 1), t1=round(float(t[b]), 1),
                         peak=round(float(seg[np.abs(seg).argmax()]), 3), lane_change=bool(is_lc),
                         completed=bool(crossed), lc_module=intent))
    return runs


# ---------------------------------------------------------------- ROS
LATCH = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                   reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
BE = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)


class Runner(Node):
    def __init__(self):
        super().__init__("beytepe_test_runner")
        self.st = dict(route=None, op=None, motion=None, mrm=None, loc=None, route_ids=[])
        self.odom = None; self.acc = 0.0
        self.rec = None; self.lock = threading.Lock()
        self.create_subscription(RouteState, "/api/routing/state", lambda m: self._s("route", m.state), LATCH)
        self.create_subscription(OperationModeState, "/api/operation_mode/state", lambda m: self._s("op", m.mode), LATCH)
        self.create_subscription(MotionState, "/api/motion/state", lambda m: self._s("motion", m.state), LATCH)
        self.create_subscription(MrmState, "/api/fail_safe/mrm_state", lambda m: self._s("mrm", (m.state, m.behavior)), LATCH)
        self.create_subscription(LocalizationInitializationState, "/api/localization/initialization_state",
                                 lambda m: self._s("loc", m.state), LATCH)
        self.create_subscription(LaneletRoute, "/planning/mission_planning/route", self._route, LATCH)
        self.create_subscription(Odometry, "/localization/kinematic_state", self._odom, BE)
        self.create_subscription(AccelWithCovarianceStamped, "/localization/acceleration", self._acc, BE)
        self.min_obj = float("nan")   # algılanan en yakın nesnenin merkezine uzaklık (yalnız e2e'de anlamlı)
        self.create_subscription(PredictedObjects, "/perception/object_recognition/objects", self._objs, BE)
        # Autoware şerit değiştirme modülünün durumu (sapma ölçütü bilinçli şerit değiştirmeyi ayırt etsin diye)
        self.lc_t = -1e9
        try:
            from tier4_rtc_msgs.msg import CooperateStatusArray
            for side in ("left", "right"):
                self.create_subscription(CooperateStatusArray, f"/planning/cooperate_status/lane_change_{side}",
                                         self._lc, 10)
        except ImportError:
            pass
        self.c_init = self.create_client(InitializeLocalization, "/api/localization/initialize")
        self.c_route = self.create_client(SetRoutePoints, "/api/routing/set_route_points")
        self.c_clear = self.create_client(ClearRoute, "/api/routing/clear_route")
        self.c_auto = self.create_client(ChangeOperationMode, "/api/operation_mode/change_to_autonomous")
        self.c_stop = self.create_client(ChangeOperationMode, "/api/operation_mode/change_to_stop")

    def _s(self, k, v):
        self.st[k] = v

    def _route(self, m):
        ids = []
        for seg in m.segments:
            ids.append([p.id for p in seg.primitives])
        self.st["route_ids"] = ids
        self.st["route_pref"] = [seg.preferred_primitive.id for seg in m.segments]

    def _objs(self, m):
        if self.odom is None:
            return
        e = self.odom.pose.pose.position
        ds = [math.hypot(o.kinematics.initial_pose_with_covariance.pose.position.x - e.x,
                         o.kinematics.initial_pose_with_covariance.pose.position.y - e.y) for o in m.objects]
        self.min_obj = min(ds) if ds else float("inf")

    def _acc(self, m):
        self.acc = m.accel.accel.linear.x

    def _lc(self, m):
        # durum: 0 bekliyor, 1 yürütülüyor, 2 iptal ediliyor, 3 bitti, 4 başarısız; araç yalnız 1-2'de yanal hareket eder
        for s in m.statuses:
            typ = s.state.type if hasattr(s.state, "type") else s.state
            if int(typ) in (1, 2):
                self.lc_t = time.time()

    def _odom(self, m):
        self.odom = m
        r = self.rec
        if r is not None:
            p = m.pose.pose.position
            now = time.time()
            r.append((now, p.x, p.y, m.twist.twist.linear.x, self.acc, m.twist.twist.angular.z,
                      self.st["op"], self.st["route"], self.st["mrm"], self.min_obj, now - self.lc_t < 1.0))

    def call(self, cli, req, timeout=15.0):
        if not cli.wait_for_service(timeout_sec=timeout):
            return None
        fut = cli.call_async(req)
        t0 = time.time()
        while not fut.done() and time.time() - t0 < timeout:
            time.sleep(0.05)
        return fut.result() if fut.done() else None


def wait(cond, timeout, dt=0.1):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if cond():
            return True
        time.sleep(dt)
    return False


def run_one(node, spec):
    res = dict(name=spec["name"], title=spec.get("title", ""), drive=spec.get("drive", True))
    e2e = bool(spec["start"].get("current"))   # AWSIM: araç ışınlanamaz, mevcut konumdan başlanır
    if e2e:
        wait(lambda: node.odom is not None, 20)
    (s_lid, s_s), (g_lid, g_s) = resolve(spec, node)
    sp, syaw = pose_at(s_lid, s_s); gp, gyaw = pose_at(g_lid, g_s)
    # isteğe bağlı ara noktalar (Autoware SetRoutePoints.waypoints): rota bu noktalardan geçerek hedefe gider
    wps = [pose_at(w["lanelet"], w.get("frac", 0.5) * length(w["lanelet"])) for w in spec.get("waypoints", [])]
    if wps:
        res["waypoints"] = [[round(float(c), 2) for c in p] for p, _ in wps]
    res.update(start_lanelet=s_lid, goal_lanelet=g_lid, start=sp.round(2).tolist(), goal=gp.round(2).tolist(),
               start_yaw_deg=round(math.degrees(syaw), 1), goal_yaw_deg=round(math.degrees(gyaw), 1))

    # 1) dur + rotayı temizle
    node.call(node.c_stop, ChangeOperationMode.Request())
    wait(lambda: node.st["op"] == OperationModeState.STOP, 10)
    wait(lambda: node.odom is not None and abs(node.odom.twist.twist.linear.x) < 0.05, 30)
    node.call(node.c_clear, ClearRoute.Request())
    wait(lambda: node.st["route"] == RouteState.UNSET, 10)

    # 2) lokalizasyon (e2e'de NDT zaten çalışıyor; yalnız durumunu kontrol et)
    if e2e:
        if node.st["loc"] != LocalizationInitializationState.INITIALIZED:
            res.update(result="FAIL", reason="lokalizasyon başlatılmamış (e2e)"); return res
        res["init_ok"] = True
    else:
        _init_localization(node, res, sp, syaw)
        if res.get("result") == "FAIL":
            return res
    return _route_and_drive(node, spec, res, gp, gyaw, wps)


def _init_localization(node, res, sp, syaw):
    req = InitializeLocalization.Request()
    pc = PoseWithCovarianceStamped(); pc.header.frame_id = "map"
    pc.header.stamp = node.get_clock().now().to_msg(); pc.pose.pose = mk_pose(sp, syaw)
    cov = [0.0] * 36; cov[0] = cov[7] = 0.25; cov[35] = 0.0685
    pc.pose.covariance = cov; req.pose = [pc]
    r = node.call(node.c_init, req, 30)
    res["init_ok"] = bool(r and r.status.success)
    res["init_msg"] = r.status.message if r else "servis yanıt vermedi"
    if not wait(lambda: node.st["loc"] == LocalizationInitializationState.INITIALIZED, 20):
        res.update(result="FAIL", reason="lokalizasyon başlatılamadı"); return
    time.sleep(2.0)
    if node.odom is not None:
        op_ = node.odom.pose.pose.position
        res["init_pos_err"] = round(math.hypot(op_.x - sp[0], op_.y - sp[1]), 3)


def _route_and_drive(node, spec, res, gp, gyaw, wps=()):
    # 3) rota
    node.st["route_ids"] = []
    req = SetRoutePoints.Request(); req.header.frame_id = "map"
    req.header.stamp = node.get_clock().now().to_msg(); req.goal = mk_pose(gp, gyaw)
    req.waypoints = [mk_pose(p, yaw) for p, yaw in wps]
    r = node.call(node.c_route, req, 30)
    res["route_ok"] = bool(r and r.status.success)
    res["route_msg"] = (r.status.message if r else "servis yanıt vermedi")
    if not res["route_ok"]:
        res.update(result="FAIL", reason="rota oluşturulamadı: " + res["route_msg"]); return res
    wait(lambda: len(node.st["route_ids"]) > 0, 10)
    res["route_lanelets"] = node.st.get("route_pref", [])
    rl = node.st.get("route_pref", [])
    res["route_length_m"] = round(float(sum(length(i) for i in rl)), 1) if rl else None
    if not spec.get("drive", True):
        res.update(result="PASS", reason="rota oluştu (sürüş yapılmadı)"); return res

    # 4) otonom
    time.sleep(3.0)
    engaged = False; t0 = time.time()
    while time.time() - t0 < 40:
        r = node.call(node.c_auto, ChangeOperationMode.Request())
        if r and r.status.success:
            engaged = True; break
        time.sleep(1.0)
    res["engage_msg"] = r.status.message if r else ""
    if not engaged:
        res.update(result="FAIL", reason="otonom moda geçilemedi: " + res["engage_msg"]); return res

    # 5) izle
    mrm_at_engage = node.st["mrm"]
    node.rec = []; t_start = time.time(); moved_t = None; last_move = time.time()
    timeout = spec.get("timeout", 300); reason = None; mrm_seen = set()
    while True:
        time.sleep(0.2)
        now = time.time(); v = abs(node.odom.twist.twist.linear.x) if node.odom else 0.0
        if node.st["mrm"] and node.st["mrm"][0] not in (0, 1):
            mrm_seen.add(str(node.st["mrm"]))
        if v > 0.3:
            last_move = now
            if moved_t is None:
                moved_t = now
        if node.st["route"] == RouteState.ARRIVED:
            reason = "arrived"; break
        if moved_t is None and now - t_start > 60:
            reason = "hiç hareket etmedi (60 s)"; break
        if moved_t is not None and now - last_move > spec.get("stuck_s", STUCK_S):
            reason = f"yolda {spec.get('stuck_s', STUCK_S):.0f} s'den uzun durdu (takıldı)"; break
        if now - t_start > timeout:
            reason = "zaman aşımı"; break
    time.sleep(1.0)
    rec = node.rec; node.rec = None
    a = np.array([r[:6] for r in rec], dtype=float) if rec else np.zeros((0, 6))
    # metrikler
    polys = [CL[i] for seg in node.st["route_ids"] for i in seg if i in CL]
    moving = a[:, 3] > 0.5 if len(a) else np.zeros(0, bool)
    devs = np.array([signed_dev(p, polys) for p in a[:, 1:3]]) if len(a) else np.zeros(0)
    lat_lim = spec.get("lat_dev_limit", 0.5)
    lcf = np.array([bool(r[10]) for r in rec], dtype=bool) if rec else np.zeros(0, bool)
    runs = lane_change_runs(a[:, 0] - a[0, 0], devs, lat_lim, lcf) if len(a) else []
    in_lc = np.zeros(len(devs), bool)
    for rr in runs:
        if rr["lane_change"]:
            in_lc[rr["a"]:rr["b"] + 1] = True
    keep = moving & ~in_lc          # şerit içi sürüş: hareket halinde ve bilinçli şerit değiştirme dışında
    fin = a[-1, 1:3] if len(a) else np.array([np.nan, np.nan])
    path_len = float(np.linalg.norm(np.diff(a[:, 1:3], axis=0), axis=1).sum()) if len(a) > 1 else 0.0
    m = dict(
        arrived=reason == "arrived", stop_reason=reason,
        drive_time_s=round(float(a[-1, 0] - (moved_t or a[0, 0])), 1) if len(a) else None,
        path_length_m=round(path_len, 1),
        max_speed_kmh=round(float(a[:, 3].max() * 3.6), 1) if len(a) else None,
        mean_speed_kmh=round(float(a[moving, 3].mean() * 3.6), 1) if moving.any() else None,
        max_accel=round(float(a[:, 4].max()), 2) if len(a) else None,
        max_decel=round(float(-a[:, 4].min()), 2) if len(a) else None,
        max_lat_accel=round(float(np.abs(a[:, 3] * a[:, 5]).max()), 2) if len(a) else None,
        lat_dev_max=round(float(np.abs(devs[moving]).max()), 3) if moving.any() else None,
        lat_dev_rms=round(float(np.sqrt((devs[moving] ** 2).mean())), 3) if moving.any() else None,
        # Yukarıdaki iki değer ham (tüm hareketli örnekler; eski sonuçlarla karşılaştırılabilir). Başarı ölçütü
        # aşağıdakini kullanır: bilinçli şerit değiştirmeler (bkz. lane_change_runs) hariç şerit içi sapma.
        lat_dev_max_lc_excl=round(float(np.abs(devs[keep]).max()), 3) if keep.any() else None,
        lat_dev_rms_lc_excl=round(float(np.sqrt((devs[keep] ** 2).mean())), 3) if keep.any() else None,
        lane_changes=[dict(t0=rr["t0"], t1=rr["t1"], peak=rr["peak"], completed=rr["completed"],
                           lc_module=rr["lc_module"]) for rr in runs if rr["lane_change"]],
        lat_dev_events=[dict(t0=rr["t0"], t1=rr["t1"], peak=rr["peak"],
                             x=round(float(a[rr["a"] + int(np.abs(devs[rr["a"]:rr["b"] + 1]).argmax()), 1]), 1),
                             y=round(float(a[rr["a"] + int(np.abs(devs[rr["a"]:rr["b"] + 1]).argmax()), 2]), 1))
                        for rr in runs if not rr["lane_change"] and moving[rr["a"]:rr["b"] + 1].any()],
        lc_module_seen=bool(lcf.any()),
        final_goal_dist=round(float(np.hypot(*(fin - gp[:2]))), 2),
        mrm=sorted(mrm_seen),
        mrm_at_engage=str(mrm_at_engage),
        # MRM aktif olan kayıtlar: (zaman s, state, behavior, hız m/s)
        mrm_events=[(round(r[0] - rec[0][0], 1), r[8][0], r[8][1], round(r[3], 2))
                    for r in rec if r[8] and r[8][0] not in (0, 1)][:400],
    )
    objd = np.array([r[9] for r in rec], dtype=float) if rec else np.zeros(0)
    fin_obj = objd[np.isfinite(objd)]
    m["min_obj_dist"] = round(float(fin_obj.min()), 2) if len(fin_obj) else None   # merkezler arası, m
    res.update(m)
    res["series"] = dict(t=(a[:, 0] - a[0, 0]).round(2).tolist() if len(a) else [],
                         x=a[:, 1].round(2).tolist(), y=a[:, 2].round(2).tolist(),
                         v=a[:, 3].round(3).tolist(), acc=a[:, 4].round(3).tolist(),
                         dev=devs.round(3).tolist(), lane_change=in_lc.astype(int).tolist(), lc=lcf.astype(int).tolist(),
                         obj=[None if not math.isfinite(d) else round(d, 2) for d in objd])
    fails = []
    if not m["arrived"]: fails.append(reason)
    if m["lat_dev_max_lc_excl"] is not None and m["lat_dev_max_lc_excl"] > lat_lim:
        fails.append(f"şerit sapması {m['lat_dev_max_lc_excl']} m > {lat_lim} (şerit değiştirmeler hariç)")
    if m["max_decel"] is not None and m["max_decel"] > 3.0: fails.append(f"yavaşlama {m['max_decel']} > 3 m/s²")
    if m["final_goal_dist"] > GOAL_TOL: fails.append(f"hedefe uzaklık {m['final_goal_dist']} m")
    if mrm_seen: fails.append("MRM tetiklendi")
    res["result"] = "PASS" if not fails else "FAIL"
    res["reason"] = "; ".join(fails) if fails else "hedefe ulaştı, ölçütler sağlandı"
    return res


def main():
    specs = json.load(open(sys.argv[1])); out = sys.argv[2]; os.makedirs(out, exist_ok=True)
    only = sys.argv[3].split(",") if len(sys.argv) > 3 else None
    rclpy.init(); node = Runner()
    ex = MultiThreadedExecutor(); ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    wait(lambda: node.odom is not None or node.st["loc"] is not None, 10)
    for spec in specs:
        if only and spec["name"] not in only:
            continue
        print(f"=== {spec['name']} başlıyor", flush=True)
        try:
            r = run_one(node, spec)
        except Exception as e:  # noqa
            import traceback; traceback.print_exc()
            r = dict(name=spec["name"], result="ERROR", reason=repr(e))
        json.dump(r, open(os.path.join(out, spec["name"] + ".json"), "w"), ensure_ascii=False)
        print(f"=== {spec['name']}: {r['result']} — {r.get('reason')} "
              f"(süre {r.get('drive_time_s')} s, sapma max {r.get('lat_dev_max_lc_excl')} m "
              f"[ham {r.get('lat_dev_max')} m, {len(r.get('lane_changes') or [])} şerit değiştirme hariç], "
              f"yavaşlama max {r.get('max_decel')}, rota {r.get('route_length_m')} m)", flush=True)
    node.call(node.c_stop, ChangeOperationMode.Request())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
