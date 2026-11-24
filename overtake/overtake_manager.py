#!/usr/bin/env python3
"""Hüsmart / Beytepe — sollama yöneticisi (overtake manager)

Autoware hareket eden bir aracı kendiliğinden sollamaz. Bu düğüm, kullanıcının kurallarıyla sollama kararını verir;
şerit değiştirmenin kendisini (yörünge + hedef şeritteki araçlara karşı RSS güvenlik kontrolü) Autoware yapar.

Karar modeli: MOBIL + IDM, asimetrik ("sağdan git") sürüm — ayrıntı ve kaynaklar aşağıda "IDM + MOBIL" bölümünde.
  * Sola geçiş (sollama): MOBIL kazancı (bizim ivme kazancımız + nezaket x sol şeritte arkamızda kalacak aracın kaybı)
    eşik + sağa eğilimden büyük VE o aracın zorunlu frenlemesi b_safe'i aşmıyor; hold_s boyunca kesintisiz.
  * Sağa dönüş: MOBIL sağa geçiş kazancı (sağa eğilim burada lehte) ve sağ şeritte arkamızda kalacak araç
    (genelde geçtiğimiz araç) için güvenlik. Geçtiğimiz aracın önüne güvensiz kesme ve sol şeridi gereksiz
    işgal etme böylece kendiliğinden önlenir; ayrıca en geç max_left_s saniyede dönüş istenir.
  * Sağdan geçme yasağı (asimetrik MOBIL): sağ şeritteyken sol şeritte önde giden daha yavaş aracı geçmemek
    için hız sınırı konur; sol şeritteyken sağ şeride dönüş kazancı da o araca göre sınırlanır.
Kural katmanı (MOBIL'in bilmediği): sol şerit şerit değiştirmeye izinli ve rotada, sollama mesafesi boyunca kavşak /
yaya geçidi / keskin viraj yok ve şerit sürüyor, sol şerit öndeki aracın ötesine kadar boş, hedefe yakın değil.
Kavşak payı (2026-11-09, 14. tur): sağa dönüş gecikebilir (Autoware "Not in lane driving state" reddi, ardışık yavaş
araç). Kavşak girişinde şerit değiştirme iptal olunca planlama "ani kayma" görüp MRM yaptı ve araç takıldı. Bu yüzden
  * sollamaya ve ardışık sollamaya ancak kavşak / şerit sonu, sollama yolundan sonra en az junction_reserve_s x hız
    (en az junction_reserve_min) uzaktaysa başlanır;
  * sol şeritteyken kavşak / şerit sonu return_lookahead_s x hız + lc_dist içine girince sağa dönülür (eskiden 60 m);
  * Autoware'in geçici reddinden sonra istek 1 s'de bir yeniden gönderilir (eskiden 3 s).
Rota sol şeridi gerektirirken sağa dönüş (2026-11-24, 24. tur): asıl rota zaten sol şeritteyse sollamadan sonra sağ şerit
keep_right_need + return_lookahead_s x hız boyunca rotada sürmüyorsa solda kalınır; başlamamış sağa dönüş, rota
keep_right_release içinde sol şeridi gerektirince iptal edilir (nizamiye U dönüşü sol şeritten yapılır).

Mekanizma (Autoware ayarı değiştirilmez)
  * Sola geçiş / geri dönüş: /planning/mission_planning/manual_lane_change_handler/set_preferred_lane
    (LEFT = tercih edilen şeridi sola kaydır, AUTO = rotanın asıl şeritlerine dön). Şerit değişimini Autoware'in
    lane_change modülü kendi güvenlik kontrolüyle yapar; güvenli değilse beklemeye devam eder.
  * Sağdan geçmeme: /planning/scenario_planning/max_velocity_candidates (sender = overtake_manager).

Çalıştırma (Autoware imajı içinde, AWSIM + Autoware açıkken):
  python3 overtake_manager.py --ros-args -p use_sim_time:=true -p map_path:=/autoware_map/lanelet2_map.osm
MOBIL örnek kararları (ROS gerekmez):
  python3 overtake_manager.py --mobil-test
Harita kontrolü (ROS gerekmez):
  python3 overtake_manager.py --check /autoware_map/lanelet2_map.osm
"""
import math
import sys

import lanelet2
from lanelet2.core import BasicPoint2d
from lanelet2.io import Origin
from lanelet2.projection import LocalCartesianProjector

# ----------------------------------------------------------------------------------------------- parametreler
P = dict(
    rate_hz=5.0,
    v_cap=50.0 / 3.6,            # rota üst hızı (planning common max_vel)
    lead_min_speed=1.5,          # bunun altı "duran araç": sollama değil, Autoware'in kaçınma modülü
    lead_max_gap=45.0,           # sollama yalnızca öndeki araç bu mesafeden yakınsa düşünülür (m)
    left_free_extra=40.0,        # sol şerit, öndeki aracın bu kadar ötesine kadar boş olmalı (m)
    pass_margin_min=8.0,         # sollama mesafesi hesabında geçilen aracın önündeki pay (m)
    min_speed_gap=1.5,           # öndeki araç istenen hızdan en az bu kadar yavaş olmalı (m/s, ~5 km/sa)
    return_min_gap=5.0, return_min_headway=0.5,  # sağa dönerken arkada kalacak araçla en az boşluk (m) / süre (s)
    keep_right_need=250.0,       # sağdan git: sağ şerit en az bu kadar rotada sürmeli (m)
    keep_right_release=200.0,    # sağdan git: rota bu mesafede sol şeridi gerektiriyorsa kaydırmayı bırak (m)
    max_left_s=25.0,             # sol şeritte en fazla (tek araç sollarken)
    max_left_chain_s=40.0,       # ardışık sollamada sol şeritte en fazla
    chain_dist=150.0,            # sol şeritteyken sağda bu mesafede yavaş araç varsa dönme (ardışık sollama, m)
    chain_clear_s=1.5,           # sağa dönmek için sağdaki yavaş araç bu kadar süre hiç görülmemiş olmalı (s)
    chain_confirm_s=0.5,         # dönüşü iptal etmek için yeni yavaş araç bu kadar kesintisiz görülmeli (s)
    cooldown_ok_s=2.0,           # başarılı sollamadan sonra bekleme (s)
    follow_gain=0.2,             # yumuşak takip: hız sınırı = öndeki hızı + kazanç x (boşluk - istenen mesafe)
    undertake_min_speed=15.0 / 3.6,  # sol şerit bundan yavaş akıyorsa (kuyruk) sağdan geçme yasağı uygulanmaz
    limit_min_acc=-1.0,          # hız sınırlarına uyarken en fazla yavaşlama (m/s²)
    change_timeout_s=15.0,       # sola geçiş bu sürede başlamazsa vazgeç (Autoware hazırlık evresi dahil)
    cooldown_s=10.0,             # iki sollama arası
    lc_dist=35.0,                # bir şerit değişimi için gereken yol (m)
    junction_reserve_s=12.0,     # kavşak / şerit sonu, sollama yolundan sonra en az bu süre x hız kadar uzakta olmalı (s)
    junction_reserve_min=120.0,  # aynı payın en küçüğü (m)
    return_lookahead_s=10.0,     # sol şeritte kavşak / şerit sonu (bu süre x hız + lc_dist) içindeyse sağa dön (s)
    retry_busy_s=1.0,            # Autoware "Not in lane driving state" derse isteği bu kadar sonra yeniden gönder (s)
    max_heading_change_deg=30.0,  # sollama mesafesinde en fazla yön değişimi (viraj)
    crosswalk_lat=6.0,           # yaya geçidinin şeride yanal yakınlığı (m)
    no_ot_before_goal=150.0,     # hedefe bu kadar kala sollama yok (m)
    lane_half_width=1.6, max_yaw_diff_deg=35.0,
    undertake_gap=5.0, undertake_gain=0.4,      # sağdan geçmeme hız sınırı: boşluk (m), kazanç (1/s)
    ego_front=3.79, ego_rear=1.1,  # base_link (arka aks) -> ön / arka uç (m)
)
MAP_ORIGIN = (39.87116440956932, 32.7356890084472)   # map_projector_info.yaml (Local; local_x/local_y kullanılır)
VEHICLE_LABELS = (1, 2, 3, 4, 5)                     # CAR, TRUCK, BUS, TRAILER, MOTORCYCLE


# ----------------------------------------------------------------------------------------------- harita
def attr(o, k, default=None):
    try:
        return o.attributes[k]
    except Exception:
        return default


class LaneMap:
    def __init__(self, path):
        m, errs = lanelet2.io.loadRobust(path, LocalCartesianProjector(Origin(*MAP_ORIGIN)))
        moved = 0
        for p in m.pointLayer:   # Autoware 'Local' projektörü: düğümlerdeki local_x / local_y / ele doğrudan
            try:
                p.x = float(p.attributes["local_x"])
                p.y = float(p.attributes["local_y"])
                moved += 1
            except Exception:
                pass
            try:
                p.z = float(p.attributes["ele"])
            except Exception:
                pass
        self.m = m
        self.moved = moved
        self.load_errors = len(errs)
        rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                              lanelet2.traffic_rules.Participants.Vehicle)
        self.g = lanelet2.routing.RoutingGraph(m, rules)
        self.road = {ll.id: ll for ll in m.laneletLayer if attr(ll, "subtype") == "road"}
        self.pts = {i: resample([(p.x, p.y) for p in ll.centerline], 2.0) for i, ll in self.road.items()}
        self.length = {i: plen(pts) for i, pts in self.pts.items()}
        self.turn = {i: attr(ll, "turn_direction") for i, ll in self.road.items()}
        self.speed = {}
        for i, ll in self.road.items():
            try:
                self.speed[i] = float(attr(ll, "speed_limit")) / 3.6
            except Exception:
                self.speed[i] = None
        self.crosswalks = []
        for ll in m.laneletLayer:
            if attr(ll, "subtype") == "crosswalk":
                xs = [p.x for p in ll.leftBound] + [p.x for p in ll.rightBound]
                ys = [p.y for p in ll.leftBound] + [p.y for p in ll.rightBound]
                self.crosswalks.append((sum(xs) / len(xs), sum(ys) / len(ys)))

    def left(self, i):      # şerit değiştirmeye izinli sol komşu
        x = self.g.left(self.road[i]) if i in self.road else None
        return x.id if x is not None and x.id in self.road else None

    def right(self, i):
        x = self.g.right(self.road[i]) if i in self.road else None
        return x.id if x is not None and x.id in self.road else None

    def adj_left(self, i):  # şerit değiştirmesi yasak olsa da soldaki aynı yönlü şerit
        x = self.left(i)
        if x is not None:
            return x
        y = self.g.adjacentLeft(self.road[i]) if i in self.road else None
        return y.id if y is not None and y.id in self.road else None

    def following(self, i):
        return [x.id for x in self.g.following(self.road[i]) if x.id in self.road]

    def previous(self, i):
        return [x.id for x in self.g.previous(self.road[i]) if x.id in self.road]

    def find_lanelet(self, x, y, yaw, prefer=()):
        best, best_score = None, 1e9
        for d, ll in lanelet2.geometry.findNearest(self.m.laneletLayer, BasicPoint2d(x, y), 8):
            if ll.id not in self.road or d > 0.8:
                continue
            s, off, hd, dist = project(self.pts[ll.id], x, y)
            dyaw = abs(angdiff(hd, yaw))
            if dyaw > math.radians(P["max_yaw_diff_deg"]):
                continue
            score = d * 5.0 + dyaw + (-0.5 if ll.id in prefer else 0.0)
            if score < best_score:
                best, best_score = ll.id, score
        return best

    def corridor(self, start, back, fwd, route_ids=()):
        def pick(cands):
            if not cands:
                return None
            if len(cands) == 1:
                return cands[0]
            r = [c for c in cands if c in route_ids]
            if r:
                return r[0]
            st = [c for c in cands if self.turn.get(c) in (None, "straight")]
            return st[0] if st else None
        seq, acc, cur = [], 0.0, start
        while acc < back:
            p = pick(self.previous(cur))
            if p is None or p in seq or p == start:
                break
            seq.insert(0, p)
            acc += self.length[p]
            cur = p
        seq.append(start)
        acc, cur = self.length[start], start
        while acc < fwd:
            n = pick(self.following(cur))
            if n is None or n in seq:
                break
            seq.append(n)
            acc += self.length[n]
            cur = n
        return Corridor(self, seq)


class Corridor:
    def __init__(self, lm, seq):
        self.seq = seq
        self.pts, self.cum, self.ranges = [], [0.0], []
        for i in seq:
            pts = lm.pts[i]
            start_s = self.cum[-1] if self.pts else 0.0
            for q in (pts if not self.pts else pts[1:]):
                if self.pts:
                    a = self.pts[-1]
                    self.cum.append(self.cum[-1] + math.hypot(q[0] - a[0], q[1] - a[1]))
                self.pts.append(q)
            self.ranges.append((i, start_s, self.cum[-1]))
        if len(self.cum) > len(self.pts):
            self.cum = self.cum[:len(self.pts)]
        self.total = self.cum[-1] if self.cum else 0.0

    def project(self, x, y):
        return project(self.pts, x, y, self.cum)

    def lanelet_at(self, s):
        for i, a, b in self.ranges:
            if a <= s <= b:
                return i
        return None

    def heading_at(self, s):
        for k in range(len(self.cum) - 1):
            if self.cum[k + 1] >= s:
                a, b = self.pts[k], self.pts[k + 1]
                return math.atan2(b[1] - a[1], b[0] - a[0])
        a, b = self.pts[-2], self.pts[-1]
        return math.atan2(b[1] - a[1], b[0] - a[0])


def plen(pts):
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))


def resample(pts, step):
    if len(pts) < 2:
        return pts
    out, carry = [pts[0]], 0.0
    for a, b in zip(pts, pts[1:]):
        seg = math.hypot(b[0] - a[0], b[1] - a[1])
        t = step - carry
        while t <= seg:
            out.append((a[0] + (b[0] - a[0]) * t / seg, a[1] + (b[1] - a[1]) * t / seg))
            t += step
        carry = seg - (t - step)
    if math.hypot(out[-1][0] - pts[-1][0], out[-1][1] - pts[-1][1]) > 1e-3:
        out.append(pts[-1])
    return out


def project(pts, x, y, cum=None):
    """(s yay uzunluğu, d yanal [+ sol], yön, uzaklık)"""
    best = (1e18, 0.0, 0.0, 0.0)
    acc = 0.0
    for k in range(len(pts) - 1):
        ax, ay = pts[k]
        bx, by = pts[k + 1]
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy
        L = math.sqrt(L2) if L2 > 0 else 1e-9
        base = cum[k] if cum is not None else acc
        if L2 > 0:
            t = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / L2))
            px, py = ax + t * dx, ay + t * dy
            dd = (x - px) ** 2 + (y - py) ** 2
            if dd < best[0]:
                best = (dd, base + t * L, (dx * (y - ay) - dy * (x - ax)) / L, math.atan2(dy, dx))
        acc += L
    return best[1], best[2], best[3], math.sqrt(best[0])


def angdiff(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def road_clear(lm, corr, s_from, dist, side="left"):
    """s_from'dan dist kadar ileride: yan şerit (side) sürekli mi, kavşak / yaya geçidi / keskin viraj var mı."""
    s_to = min(s_from + dist, corr.total)
    if corr.total - s_from < dist:
        return False, "şerit/rota bitiyor"
    for i, a, b in corr.ranges:
        if b < s_from or a > s_to:
            continue
        if lm.turn.get(i) is not None:
            return False, "kavşak"
        if (lm.left(i) if side == "left" else lm.right(i)) is None:
            return False, ("sol" if side == "left" else "sağ") + " şerit yok/değiştirme yasak"
    for cx, cy in lm.crosswalks:
        s, d, _, _ = corr.project(cx, cy)
        if s_from - 5 < s < s_to and abs(d) < P["crosswalk_lat"]:
            return False, "yaya geçidi"
    h0 = corr.heading_at(s_from)
    s = s_from
    while s < s_to:
        if abs(angdiff(corr.heading_at(s), h0)) > math.radians(P["max_heading_change_deg"]):
            return False, "viraj"
        s += 5.0
    return True, ""


def junction_dist(lm, corr, s_from, dist):
    """s_from'dan dist kadar ileride ilk kavşak lanelet'inin (turn_direction etiketli) ya da koridor / rota sonunun
    uzaklığı (m); ikisi de yoksa None. Kavşağın içindeysek 0."""
    s_to = s_from + dist
    for i, a, b in corr.ranges:
        if b < s_from or a > s_to:
            continue
        if lm.turn.get(i) is not None:
            return max(0.0, a - s_from)
    if corr.total < s_to:
        return max(0.0, corr.total - s_from)
    return None


def junction_reserve(v_des):
    """Kavşaktan önce sağa dönüş için bırakılan pay (m)."""
    return max(P["junction_reserve_min"], v_des * P["junction_reserve_s"])


def check_map(path):
    lm = LaneMap(path)
    two = [i for i in lm.road if lm.left(i) is not None]
    print(f"harita: {len(lm.road)} road lanelet, koordinatı düzeltilen nokta {lm.moved}, yükleme uyarısı {lm.load_errors}, "
          f"yaya geçidi {len(lm.crosswalks)}; sol şeride geçilebilen lanelet {len(two)}")
    for dist in (120.0, 170.0, 220.0):
        ok_len, tot = 0.0, 0.0
        for i in two:
            corr = lm.corridor(i, 0.0, dist + 10)
            L = lm.length[i]
            tot += L
            ok, _ = road_clear(lm, corr, 0.0, dist)
            if ok:
                ok_len += L
        print(f"  {dist:.0f} m'lik sollama mesafesi: uygun başlangıç uzunluğu {ok_len:.0f} m / {tot:.0f} m")


# ----------------------------------------------------------------------------------------------- IDM + MOBIL
# IDM   : Treiber, Hennecke, Helbing (2000), "Congested traffic states in empirical observations and
#         microscopic simulations", Physical Review E 62.
# MOBIL : Kesting, Treiber, Helbing (2007), "General lane-changing model MOBIL for car-following models",
#         Transportation Research Record 1999.
# Asimetrik (Avrupa) kurallar — "sağdan git" eğilimi ve sağdan geçme yasağı: Treiber & Kesting (2013),
#         "Traffic Flow Dynamics", MOBIL bölümü.
#   Sola geçiş : ã_c - a_c + p (ã_n - a_n) > Δa_th + Δa_bias
#   Sağa dönüş : ã_c - a_c + p (ã_o - a_o) > Δa_th - Δa_bias
#   Güvenlik   : ã_n >= -b_safe      (hedef şeritte arkamızda kalacak araç bu kadardan sert fren yapmak zorunda kalmamalı)
#   c = biz, n = hedef şeritteki yeni arkadaki, o = şu anki şeritteki arkadaki; ~ = şerit değiştirdikten sonra.
#   Sağdan geçme yasağı: sağ şeritteyken ivmemiz, sol şeritte önde giden araca göre IDM ivmesiyle sınırlanır.
M = dict(
    T=2.0,        # istenen zaman aralığı (s) — Autoware'in takip mesafesine yakın seçildi
    s0=2.0,       # durma mesafesi (m)
    a=1.0,        # en yüksek ivme (m/s²)
    b=1.5,        # rahat yavaşlama (m/s²)
    delta=4.0,    # ivme üssü
    p=0.25,       # nezaket katsayısı (0 = bencil, 1 = tamamen düşünceli)
    a_th=0.1,     # kazanç eşiği (m/s²)
    a_bias=0.3,   # sağa eğilim (m/s²): sola geçişi zorlaştırır, sağa dönüşü kolaylaştırır
    b_safe=2.0,   # güvenli fren sınırı (m/s²); makalede 4, kampüs aracı için daha temkinli
    hold_s=1.5,   # sola geçiş kararı bu kadar süre sürekli doğru kalmalı (algı gürültüsüne karşı)
    hold_ret_s=0.6,  # sağa dönüş kararı için
    follow_cap_gaps=2.5,  # öndeki bu kadar "istenen mesafe" (s0 + v T) içindeyse ve hızımız ona eşitse a_c <= 0 sayılır
)


def idm_acc(v, v0, lead_v=None, gap=None):
    """IDM ivmesi: önünde (lead_v hızında, gap tampon-tampon uzaklıkta) araç varken ya da boş yolda."""
    v0 = max(v0, 0.1)
    free = 1.0 - (max(v, 0.0) / v0) ** M["delta"]
    if lead_v is None or gap is None:
        return M["a"] * free
    gap = max(gap, 0.1)
    s_star = M["s0"] + max(0.0, v * M["T"] + v * (v - lead_v) / (2.0 * math.sqrt(M["a"] * M["b"])))
    return max(-9.0, M["a"] * (free - (s_star / gap) ** 2))


def split_lane(objs):
    """Şeritteki araçlar -> (öndeki, arkadaki, yanımızdaki(ler)). objs: lane_objects çıktısı."""
    ahead = [o for o in objs if o["gap_front"] >= 0.0]
    behind = [o for o in objs if o["gap_back"] >= 0.0]
    overlap = [o for o in objs if o["gap_front"] < 0.0 and o["gap_back"] < 0.0]
    lead = min(ahead, key=lambda o: o["gap_front"]) if ahead else None
    foll = min(behind, key=lambda o: o["gap_back"]) if behind else None
    return lead, foll, overlap


def gap_between(f, l):
    """Aynı şeritte arkadaki f ile öndeki l arasındaki tampon-tampon uzaklık."""
    return (l["rel"] - l["L"] / 2.0) - (f["rel"] + f["L"] / 2.0)


def mobil(v, v0, cur, tgt, to_left, no_pass_lead=None):
    """MOBIL kararı. cur = (önümüzdeki, arkamızdaki) bu şeritte; tgt = aynısı hedef şeritte.
    Dönüş: dict(ok, gain, need, at_n, safe)"""
    Lc, Fo = cur
    Lt, Fn = tgt
    a_c = idm_acc(v, v0, Lc["v"], Lc["gap_front"]) if Lc else idm_acc(v, v0)
    # Yavaş öndekinin arkasında, hızı ona eşitlenmiş takip: Autoware'in takip mesafesi (~3 s) IDM dengesinden
    # (s0 + v T) uzun. IDM bu fazla boşluğu "kendi şeridimde hâlâ hızlanabilirim" sanıp a_c > 0 verir ve sollama
    # kazancını eşiğin altına çeker (30'luk aracın 27 m arkasında a_c = +0.39 -> kazanç ~0.38 < 0.40, araç dakikalarca
    # sollamadan bekledi). Gerçekte öndekinden hızlı gidemeyiz: kazanç hesabında a_c en çok 0.
    if (Lc and Lc["v"] < v0 - 0.5 and v <= Lc["v"] + 1.0
            and Lc["gap_front"] < M["follow_cap_gaps"] * (M["s0"] + Lc["v"] * M["T"])):
        a_c = min(a_c, 0.0)
    at_c = idm_acc(v, v0, Lt["v"], Lt["gap_front"]) if Lt else idm_acc(v, v0)
    if no_pass_lead is not None:      # sağdan geçme yasağı (sağ şeride geçince bu aracı sağdan geçemeyiz)
        at_c = min(at_c, idm_acc(v, v0, no_pass_lead["v"], max(no_pass_lead["gap_front"], 0.1)))
    if Fn:
        a_n = idm_acc(Fn["v"], v0, Lt["v"], gap_between(Fn, Lt)) if Lt else idm_acc(Fn["v"], v0)
        at_n = idm_acc(Fn["v"], v0, v, Fn["gap_back"])
    else:
        a_n = at_n = 0.0
    if Fo:
        a_o = idm_acc(Fo["v"], v0, v, Fo["gap_back"])
        at_o = idm_acc(Fo["v"], v0, Lc["v"], gap_between(Fo, Lc)) if Lc else idm_acc(Fo["v"], v0)
    else:
        a_o = at_o = 0.0
    if to_left:
        gain, need = at_c - a_c + M["p"] * (at_n - a_n), M["a_th"] + M["a_bias"]
    else:
        gain, need = at_c - a_c + M["p"] * (at_o - a_o), M["a_th"] - M["a_bias"]
    safe = at_n >= -M["b_safe"]
    return dict(ok=bool(safe and gain > need), gain=gain, need=need, at_n=at_n, safe=safe)


def mobil_selftest():
    """ROS gerekmeden MOBIL kararlarını örnek durumlarda gösterir."""
    def car(rel, v, L=4.5, ego_v=None):
        gf = (rel - L / 2) - P["ego_front"]
        gb = -P["ego_rear"] - (rel + L / 2)
        return dict(rel=rel, v=v, L=L, gap_front=gf, gap_back=gb)
    kmh = 1 / 3.6
    v0 = 50 * kmh
    cases = [
        ("30'luk aracın 20 m arkasında, sol şerit boş", 30 * kmh, (car(28.3, 30 * kmh), None), (None, None), True, None),
        ("aynı + sol şeritte 15 m arkadan 50'lik araç", 30 * kmh, (car(28.3, 30 * kmh), None), (None, car(-20, 50 * kmh)), True, None),
        ("aynı + sol şeritte 60 m arkada 50'lik araç", 30 * kmh, (car(28.3, 30 * kmh), None), (None, car(-65, 50 * kmh)), True, None),
        ("45'lik aracın 25 m arkasında (az fark)", 45 * kmh, (car(33.3, 45 * kmh), None), (None, None), True, None),
        ("sol şeritte, geçilen 30'luk araç 3 m arkada", 50 * kmh, (None, None), (None, car(-9.5, 30 * kmh)), False, None),
        ("sol şeritte, geçilen 30'luk araç 12 m arkada", 50 * kmh, (None, None), (None, car(-18.5, 30 * kmh)), False, None),
        ("sol şeritte, sağ şeritte 15 m önde 30'luk araç", 50 * kmh, (None, None), (car(23.3, 30 * kmh), car(-18.5, 30 * kmh)), False, None),
    ]
    for name, v, cur, tgt, left, npl in cases:
        r = mobil(v, v0, cur, tgt, left, npl)
        print(f"{name:48s}: {'GEÇ' if r['ok'] else 'bekle':5s} kazanç {r['gain']:+.2f} (gereken {r['need']:+.2f}) "
              f"yeni arkadakinin ivmesi {r['at_n']:+.2f} {'güvenli' if r['safe'] else 'GÜVENSİZ'}")


# ----------------------------------------------------------------------------------------------- ROS düğümü
def run_node():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from nav_msgs.msg import Odometry
    from autoware_perception_msgs.msg import PredictedObjects
    from autoware_planning_msgs.msg import LaneletRoute
    from autoware_adapi_v1_msgs.msg import OperationModeState, RouteState
    from autoware_internal_planning_msgs.msg import VelocityLimit, VelocityLimitClearCommand
    from tier4_external_api_msgs.srv import SetPreferredLane

    class OvertakeManager(Node):
        def __init__(self):
            super().__init__("overtake_manager")
            path = self.declare_parameter("map_path", "/autoware_map/lanelet2_map.osm").value
            self.lm = LaneMap(path)
            self.get_logger().info(f"harita yüklendi: {len(self.lm.road)} road lanelet ({path}); karar modeli MOBIL+IDM {M}")
            tl = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
            self.create_subscription(Odometry, "/localization/kinematic_state", self.on_odom, 1)
            self.create_subscription(PredictedObjects, "/perception/object_recognition/objects", self.on_objects, 1)
            self.create_subscription(LaneletRoute, "/planning/mission_planning/route", self.on_route, tl)
            self.create_subscription(OperationModeState, "/api/operation_mode/state", self.on_opmode, tl)
            self.create_subscription(RouteState, "/api/routing/state", self.on_route_state, tl)
            from tier4_rtc_msgs.msg import CooperateStatusArray
            self.lc = {"left": None, "right": None}       # (zaman, durum, güvenli)
            for side in ("left", "right"):
                self.create_subscription(CooperateStatusArray, f"/planning/cooperate_status/lane_change_{side}",
                                         lambda m, sd=side: self.on_lc_status(sd, m), 10)
            self.vel_pub = self.create_publisher(VelocityLimit, "/planning/scenario_planning/max_velocity_candidates", tl)
            self.clear_pub = self.create_publisher(VelocityLimitClearCommand, "/planning/scenario_planning/clear_velocity_limit", tl)
            self.cli = self.create_client(SetPreferredLane, "/planning/mission_planning/manual_lane_change_handler/set_preferred_lane")
            self.SetPreferredLane, self.VelocityLimit, self.VelocityLimitClearCommand = SetPreferredLane, VelocityLimit, VelocityLimitClearCommand
            self.ego = None
            self.objs = []
            self.route_ids, self.preferred_ids, self.goal_lanelet = set(), set(), None
            self.autonomous, self.route_state = False, 0
            self.state, self.t_state = "TAKIP", 0.0
            self.ok_since, self.bad_since, self.cooldown_until = None, None, 0.0
            self.target_uuid = None
            self.ego_ll = None
            self.limit_active, self.last_limit, self.last_limit_t, self.limit_why = False, None, 0.0, ""
            self.last_log, self.pending, self.lane_result, self.lane_msg = 0.0, False, None, ""
            # İstenen tercih şeridi (0 = SOL, 1 = SAĞ, 2 = ASIL ROTA). Autoware şerit değişimi sürerken isteği reddedebilir;
            # kabul edilene kadar yeniden gönderilir. Açılışta ASIL ROTA: önceki çalışmadan kalan kaydırmayı sıfırla.
            self.desired, self.confirmed, self.last_req_t, self.req_why = 2, None, -1e9, "açılış: önceki kaydırmayı sıfırla"
            self.orig_corr, self.start_ll, self.rejected_ll, self.rejected_right_ll = None, None, None, None
            # Rotanın KAYDIRILMAMIŞ (asıl) tercih şeritleri: komutlar buna göre seçilir (bkz. want_side).
            self.orig_segs, self.route_uuid = [], None
            self.last_mobil, self.return_kind, self.chain_logged, self.return_forced = None, "sollama", None, False
            self.chain_seen_t, self.chain_since = -1e9, None
            self.stats = dict(started=0, completed=0, aborted=0, keep_right=0)
            self.create_timer(1.0 / P["rate_hz"], self.step)

        # ---- girişler
        def now(self):
            return self.get_clock().now().nanoseconds * 1e-9

        def on_odom(self, m):
            q = m.pose.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            self.ego = (m.pose.pose.position.x, m.pose.pose.position.y, yaw, m.twist.twist.linear.x)

        def on_objects(self, m):
            objs = []
            for o in m.objects:
                lab = o.classification[0].label if o.classification else 0
                if lab not in VEHICLE_LABELS:
                    continue
                pz = o.kinematics.initial_pose_with_covariance.pose
                q = pz.orientation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                tw = o.kinematics.initial_twist_with_covariance.twist.linear
                objs.append(dict(x=pz.position.x, y=pz.position.y, yaw=yaw, v=math.hypot(tw.x, tw.y),
                                 L=max(o.shape.dimensions.x, 1.0), id=bytes(o.object_id.uuid)))
            self.objs = objs

        def on_route(self, m):
            ids, pref = set(), set()
            for seg in m.segments:
                pref.add(seg.preferred_primitive.id)
                for p in seg.primitives:
                    ids.add(p.id)
            self.route_ids, self.preferred_ids = ids, pref
            self.goal_lanelet = m.segments[-1].preferred_primitive.id if m.segments else None
            segs = [({p.id for p in seg.primitives}, seg.preferred_primitive.id) for seg in m.segments]
            uid = bytes(m.uuid.uuid)
            if uid != self.route_uuid or (self.desired == 2 and self.confirmed == 2 and not self.pending):
                self.orig_segs, self.route_uuid = segs, uid    # yeni rota ya da kaydırma yokken gelen rota = asıl

        def orig_side_at(self, ll):
            """Asıl rotanın bu bölümde tercih ettiği şerit 'left' mi 'right' mı (2 şeritli yolda); bilinmiyorsa None."""
            for prims, pref in self.orig_segs:
                if ll in prims and len(prims) >= 2:
                    r, l = self.lm.right(pref), self.lm.left(pref)
                    if r is not None and r in prims:
                        return "left"
                    if l is not None and l in prims:
                        return "right"
            return None

        def want_side(self, side, orig_side, why):
            """Hedef şeridi ('left'/'right') manual_lane_change_handler komutuna çevir: asıl tercih zaten o taraftaysa
            ASIL ROTA (2), değilse SOL (0) / SAĞ (1) kaydırması."""
            cmd = 2
            if side == "left" and orig_side == "right":
                cmd = 0
            elif side == "right" and orig_side == "left":
                cmd = 1
            self.want_lane(cmd, why)

        def on_lc_status(self, side, m):
            if m.statuses:
                st = m.statuses[0]
                typ = st.state.type if hasattr(st.state, "type") else st.state
                self.lc[side] = (self.now(), int(typ), bool(st.safe))

        def lc_state(self, side):
            """Autoware'in o yöndeki şerit değiştirme modülü: (durum adı, güvenli mi) ya da None (aday yok)."""
            v = self.lc.get(side)
            if v is None or self.now() - v[0] > 1.0:
                return None
            names = {0: "BEKLIYOR", 1: "YURUTULUYOR", 2: "IPTAL_EDILIYOR", 3: "BITTI", 4: "BASARISIZ"}
            return names.get(v[1], str(v[1])), v[2]

        def on_opmode(self, m):
            self.autonomous = (m.mode == 2 and m.is_autoware_control_enabled)

        def on_route_state(self, m):
            self.route_state = m.state

        # ---- çıkışlar
        def set_lane(self, direction, why):
            if not self.cli.service_is_ready():
                self.get_logger().warn("set_preferred_lane servisi hazır değil")
                return False
            req = self.SetPreferredLane.Request()
            req.lane_change_direction = direction
            fut = self.cli.call_async(req)
            name = {0: "SOL", 1: "SAĞ", 2: "ASIL ROTA"}[direction]
            self.pending = True
            self.lane_result = None

            def done(f):
                self.pending = False
                try:
                    st = f.result().status
                    self.lane_result = (direction, bool(st.success))
                    self.lane_msg = st.message
                    if st.success:
                        self.confirmed = direction
                    elif "Not in lane driving state" in st.message:
                        # şerit değişimi / başka senaryo sürüyor: kısa süre sonra tekrar dene (sync_lane en az 1 s bekler)
                        self.last_req_t = self.now() + P["retry_busy_s"] - 1.0
                    self.get_logger().info(f"tercih şeridi -> {name} ({why}): {'tamam' if st.success else 'RED'} {st.message}")
                except Exception as e:
                    self.lane_result = (direction, False)
                    self.get_logger().warn(f"set_preferred_lane hatası: {e}")
            fut.add_done_callback(done)
            return True

        def want_lane(self, direction, why):
            """İstenen tercih şeridini ayarla; gönderme / yeniden deneme step() içinde."""
            if self.desired != direction:
                self.desired, self.req_why, self.last_req_t = direction, why, -1e9

        def sync_lane(self, t):
            if self.desired == self.confirmed or self.pending or t - self.last_req_t < 1.0:
                return
            self.last_req_t = t
            self.set_lane(self.desired, self.req_why)

        def set_limit(self, v, why=""):
            t = self.now()
            if v is None:
                if self.limit_active:
                    c = self.VelocityLimitClearCommand()
                    c.stamp = self.get_clock().now().to_msg()
                    c.command, c.sender = True, "overtake_manager"
                    self.clear_pub.publish(c)
                    self.limit_active, self.last_limit, self.limit_why = False, None, ""
                    self.get_logger().info("hız sınırı kaldırıldı")
                return
            v = max(0.0, v)
            if self.limit_active and self.last_limit is not None and abs(v - self.last_limit) < 0.3 and t - self.last_limit_t < 1.0:
                return
            msg = self.VelocityLimit()
            msg.stamp = self.get_clock().now().to_msg()
            msg.max_velocity, msg.sender = float(v), "overtake_manager"
            # Yumuşak uygula: sert fren değil, en fazla 1 m/s² yavaşlama ve düşük sarsıntı (Autoware hız yumuşatıcısı)
            msg.use_constraints = True
            msg.constraints.max_acceleration, msg.constraints.min_acceleration = 1.0, P["limit_min_acc"]
            msg.constraints.max_jerk, msg.constraints.min_jerk = 0.5, -0.5
            self.vel_pub.publish(msg)
            if not self.limit_active or why.split(":")[0] != self.limit_why.split(":")[0]:
                self.get_logger().info(f"hız sınırı ({why}): {v * 3.6:.0f} km/sa")
            self.limit_active, self.last_limit, self.last_limit_t, self.limit_why = True, v, t, why

        def goto(self, st, why=""):
            ex, ey = (self.ego[0], self.ego[1]) if self.ego else (0, 0)
            self.get_logger().info(f"[{self.state} -> {st}] ({ex:.0f},{ey:.0f}) {why}")
            self.state, self.t_state = st, self.now()
            self.ok_since = self.bad_since = None

        # ---- yardımcılar
        def lane_objects(self, corr, s_ego):
            out = []
            hw = P["lane_half_width"]
            for o in self.objs:
                if abs(o["x"] - self.ego[0]) > 150 or abs(o["y"] - self.ego[1]) > 150:
                    continue
                s, d, hd, dist = corr.project(o["x"], o["y"])
                if abs(d) > hw or abs(angdiff(o["yaw"], hd)) > math.radians(45):
                    continue
                o2 = dict(o)
                o2["rel"] = s - s_ego                      # merkez - base_link (m)
                o2["gap_front"] = (s - o["L"] / 2) - (s_ego + P["ego_front"])   # o önümüzdeyse aradaki boşluk
                o2["gap_back"] = (s_ego - P["ego_rear"]) - (s + o["L"] / 2)     # o arkamızdaysa aradaki boşluk
                out.append(o2)
            return out

        @staticmethod
        def fmt(r):
            return f"MOBIL kazanç {r['gain']:+.2f}/{r['need']:+.2f} m/s², yeni arkadaki {r['at_n']:+.2f} m/s²"

        def hold(self, cond, t, dur):
            """cond en az dur saniye kesintisiz doğru mu?"""
            if not cond:
                self.ok_since = None
                return False
            if self.ok_since is None:
                self.ok_since = t
            return t - self.ok_since >= dur

        # ---- ana döngü
        def step(self):
            t = self.now()
            if self.ego is None:
                return
            x, y, yaw, v = self.ego
            active = self.autonomous and self.route_state in (2, 4)   # 2 = SET, 4 = CHANGING (tercih şeridi değişirken)
            prefer = set()
            if self.ego_ll is not None:
                prefer = {self.ego_ll, *self.lm.following(self.ego_ll)}
            ego_ll = self.lm.find_lanelet(x, y, yaw, prefer)
            self.ego_ll = ego_ll
            if not active or ego_ll is None:
                self.set_limit(None)
                if self.state != "TAKIP" and not active:
                    self.want_lane(2, "otonom/rota bitti")
                    self.goto("TAKIP", "otonom değil / rota yok")
                return
            self.sync_lane(t)
            corr = self.lm.corridor(ego_ll, 80.0, 260.0, self.route_ids)
            s_ego, _, _, _ = corr.project(x, y)
            v_des = min(P["v_cap"], self.lm.speed.get(ego_ll) or P["v_cap"])
            cobjs = self.lane_objects(corr, s_ego)
            Lc, Fo, _ = split_lane(cobjs)
            left_id, right_id = self.lm.left(ego_ll), self.lm.right(ego_ll)
            adj_left = self.lm.adj_left(ego_ll)
            lcorr = self.lm.corridor(adj_left, 80.0, 260.0) if adj_left is not None else None
            lobjs = self.lane_objects(lcorr, lcorr.project(x, y)[0]) if lcorr else []
            rcorr = self.lm.corridor(right_id, 80.0, P["chain_dist"] + 60.0) if right_id is not None else None
            robjs = self.lane_objects(rcorr, rcorr.project(x, y)[0]) if rcorr else []

            # --- hız sınırları (en düşüğü uygulanır; hiçbiri öndeki aracın hızının altına inmez -> sert fren yok)
            lims = []
            # (a) sağdan geçme yasağı (asimetrik MOBIL): sağ şeritteyken sol şeritte önde giden, bizden yavaş aracı
            #     sağdan geçme. Sol şerit çok yavaş akıyorsa (kuyruk) uygulanmaz: o zaman sağdan ilerlemek serbest.
            if self.state in ("TAKIP", "SOLA_GECIS") and not (right_id is not None and left_id is None):
                for o in lobjs:
                    if o["v"] < P["undertake_min_speed"] or o["gap_front"] < -(o["L"] + 1.0) or o["gap_front"] > 60:
                        continue    # kuyruk / duran araç, zaten yanımızda-geride ya da uzak
                    if o["v"] >= v_des - 0.5:
                        continue
                    lims.append((o["v"] + P["undertake_gain"] * max(0.0, o["gap_front"] - P["undertake_gap"]),
                                 f"sağdan geçmeme: solda {o['v'] * 3.6:.0f} km/sa araç"))
            # (b) yumuşak takip (IDM'in istenen mesafesi s0 + v*T): öndeki yavaş araca 50 ile yaklaşıp sonra sert fren
            #     yapmak yerine erkenden hızı düşür; istenen mesafeye öndekinin hızında varılır.
            if Lc is not None and P["lead_min_speed"] < Lc["v"] < v_des - 0.5:
                d_des = M["s0"] + Lc["v"] * M["T"]
                lims.append((Lc["v"] + P["follow_gain"] * max(0.0, Lc["gap_front"] - d_des),
                             f"yumuşak takip: önde {Lc['gap_front']:.0f} m'de {Lc['v'] * 3.6:.0f} km/sa araç"))
            best = min(lims, key=lambda a: a[0]) if lims else None
            if best is not None and best[0] < v_des - 0.3:
                self.set_limit(best[0], best[1])
            else:
                self.set_limit(None)

            goal_dist = None
            if self.goal_lanelet is not None and self.goal_lanelet in corr.seq:
                for i, a, b in corr.ranges:
                    if i == self.goal_lanelet:
                        goal_dist = b - s_ego

            if t - self.last_log > 5.0:
                self.last_log = t
                ld = f"önde {Lc['gap_front']:.0f} m {Lc['v'] * 3.6:.0f} km/sa" if Lc else "önde araç yok"
                self.get_logger().info(f"durum {self.state} | {v * 3.6:.0f} km/sa lanelet {ego_ll} sol {left_id} | {ld} | sol şeritte {len(lobjs)} araç"
                                       + (f" | {self.fmt(self.last_mobil)}" if self.last_mobil else "")
                                       + (f" | hız sınırı {self.last_limit * 3.6:.0f}" if self.limit_active else ""))
            self.last_mobil = None

            # --- durum makinesi
            orig_side = self.orig_side_at(ego_ll)
            in_left = right_id is not None and left_id is None      # 2 şeritli yolun sol şeridindeyiz
            in_right = left_id is not None                          # sağ şeritteyiz (solumuzda geçilebilir şerit var)
            # Kalıcı ret (rota yapısı izin vermiyor) ile geçici ret (şerit değişimi sürüyor) ayrı tutulur;
            # geçici retlerde sync_lane isteği birkaç saniye sonra yeniden gönderir.
            rejected = (self.lane_result is not None and self.lane_result[1] is False
                        and self.lane_result[0] == self.desired and "Not in lane driving state" not in self.lane_msg)

            if self.state == "TAKIP":
                if in_left:
                    self.keep_right(t, x, y, v, v_des, Lc, Fo, robjs, ego_ll, right_id, orig_side)
                    return
                if in_right and self.desired == 1 and self.confirmed == 1 and not self.pending:
                    # "Sağdan git" kaydırması etkin (asıl rota sol şeridi tercih ediyordu). Rota ileride sol şeridi
                    # gerektiriyorsa ya da asıl tercih artık sağ şeritse kaydırmayı bırak: Autoware zamanında sola geçer.
                    if orig_side != "left":
                        self.want_lane(2, "sağdan git: asıl rota artık sağ şeritte")
                    elif not self.lane_in_route_ahead(corr, s_ego, P["keep_right_release"]):
                        self.want_lane(2, f"sağdan git bitti: rota {P['keep_right_release']:.0f} m içinde sol şeridi gerektiriyor")
                # sollama (yalnızca sağ şeritteyken)
                if (Lc is None or Lc["v"] < P["lead_min_speed"] or Lc["gap_front"] > P["lead_max_gap"]
                        or Lc["v"] > v_des - P["min_speed_gap"] or left_id is None):
                    self.ok_since = None
                    return
                Lt, Fn, ov = split_lane(lobjs)
                r = mobil(v, v_des, (Lc, Fo), (Lt, Fn), to_left=True)
                self.last_mobil = r
                if not self.hold(r["ok"] and not ov, t, M["hold_s"]):
                    return
                if t < self.cooldown_until or self.pending or self.desired != self.confirmed:
                    return
                why = self.can_overtake(corr=corr, s_ego=s_ego, v_des=v_des, lead=Lc, left_id=left_id, lobjs=lobjs, goal_dist=goal_dist)
                if why:
                    if t - self.last_log > 4.9:
                        self.get_logger().info(f"sollama yok: {why} ({self.fmt(r)})")
                    return
                self.want_side("left", orig_side, f"öndeki araç {Lc['v'] * 3.6:.0f} km/sa, {Lc['gap_front']:.0f} m; {self.fmt(r)}")
                self.lane_result = None
                self.target_uuid, self.orig_corr, self.start_ll = Lc["id"], corr, ego_ll
                self.stats["started"] += 1
                self.goto("SOLA_GECIS", f"öndeki {Lc['v'] * 3.6:.0f} km/sa aracı sollama başladı ({self.fmt(r)})")
                return

            if self.state == "SOLA_GECIS":
                if in_left:                                       # sol şeride geçtik
                    self.goto("GECIS", "sol şeritteyiz")
                    return
                # Şerit değişimi başladı mı (asıl şeridin merkezinden yana kayma)? Başladıysa yalnızca zaman aşımında vazgeç.
                lat = abs(self.orig_corr.project(x, y)[1]) if self.orig_corr is not None else 0.0
                lcs = self.lc_state("left")
                started = lat > 0.5 or (lcs is not None and lcs[0] == "YURUTULUYOR")
                reason = ""
                if rejected:
                    reason = "Autoware sola geçiş isteğini reddetti"
                    self.rejected_ll = self.start_ll     # bu lanelette tekrar isteme (ör. rota sonu bağlantısı)
                elif t - self.t_state > P["change_timeout_s"] + (15.0 if started else 0.0):
                    lcs_txt = (f"Autoware şerit değiştirme: {lcs[0]}, {'güvenli' if lcs[1] else 'hedef şerit GÜVENLİ DEĞİL'}"
                               if lcs else "Autoware şerit değiştirme adayı üretmedi")
                    reason = f"sola geçiş tamamlanmadı (zaman aşımı; {lcs_txt})"
                elif not started and left_id is not None:
                    Lt, Fn, ov = split_lane(lobjs)
                    r = mobil(v, v_des, (Lc, Fo), (Lt, Fn), to_left=True)
                    self.last_mobil = r
                    # Karar verildikten sonra kazanç eşiğin hemen altına inince vazgeçmek kararı titretir (histerezis):
                    # güvenlik bozulursa ya da kazanç gerçekten kalmazsa (< 0) vazgeç.
                    bad = ov or not r["safe"] or r["gain"] < 0.0
                    if not bad:
                        self.bad_since = None
                    elif self.bad_since is None:
                        self.bad_since = t
                    elif t - self.bad_since > 1.5:
                        reason = ("sol şeritte yanımızda araç" if ov else
                                  ("MOBIL: güvenli değil" if not r["safe"] else "MOBIL: kazanç kalmadı")) + f" ({self.fmt(r)})"
                if reason:
                    self.want_side("right", orig_side, reason)
                    self.stats["aborted"] += 1
                    self.cooldown_until = t + P["cooldown_s"]
                    self.goto("TAKIP", "vazgeçildi: " + reason)
                return

            if self.state == "GECIS":
                # Asıl rota bu bölümde zaten sol şeritteyse sağa dönüş yalnızca "sağdan git" eğilimidir. Sağ şerit
                # keep_right_need + return_lookahead_s x hız boyunca rotada sürmüyorsa (ör. nizamiye U dönüşü sol şeritten)
                # solda kalınır, sollama burada biter. (Tur 24: 320 m kala yavaş aracın arkasına dönüş istendi, şerit
                # değişimi çatala kadar sürdü, araç rotada olmayan sağ çıkış şeridine girip takıldı.)
                if orig_side == "left" and right_id is not None:
                    need_r = P["keep_right_need"] + v * P["return_lookahead_s"]
                    rc = self.lm.corridor(right_id, 0.0, need_r + 20.0, self.route_ids)
                    if not self.lane_in_route_ahead(rc, rc.project(x, y)[0], need_r):
                        self.want_lane(2, "sollama solda bitti: rota ileride sol şeridi gerektiriyor")
                        self.stats["completed"] += 1
                        self.cooldown_until = t + P["cooldown_ok_s"]
                        self.goto("TAKIP", f"sollama tamam, solda kalındı: sağ şerit {need_r:.0f} m içinde rotadan ayrılıyor "
                                           f"(sollama başlayan {self.stats['started']}, biten {self.stats['completed']}, "
                                           f"vazgeçilen {self.stats['aborted']}; sağa geçiş {self.stats['keep_right']})")
                        return
                # Sol şeritteyiz. Sağa dönüş (asimetrik MOBIL): kazanç > Δa_th - Δa_bias ve sağ şeritte arkamızda
                # kalacak araç (çoğu zaman geçtiğimiz araç) için ã_n >= -b_safe. Sol şeritte önde yavaş araç varsa
                # sağdan geçme yasağı nedeniyle sağ şeritteki ivmemiz ona göre sınırlanır.
                Lt, Fn, ov = split_lane(robjs)
                npl = Lc if (Lc is not None and Lc["v"] < v - 0.5) else None
                r = mobil(v, v_des, (Lc, Fo), (Lt, Fn), to_left=False, no_pass_lead=npl)
                self.last_mobil = r
                # Kural (trafik kuralı / konfor): MOBIL hızla uzaklaştığımız için çok yakına dönmeyi "güvenli" sayabilir;
                # geçilen aracın önüne en az max(return_min_gap, return_min_headway x onun hızı) boşlukla dön.
                gap_ok = Fn is None or Fn["gap_back"] >= max(P["return_min_gap"], P["return_min_headway"] * Fn["v"])
                road_ok, why = road_clear(self.lm, corr, s_ego, 60.0, side="right") if right_id is not None else (False, "sağ şerit bitiyor")
                if road_ok:
                    # Kavşak / şerit sonu: dönüş için hazırlık + şerit değişimi + olası Autoware reddi süresi kadar önceden dön.
                    look = max(60.0, v * P["return_lookahead_s"] + P["lc_dist"])
                    jd = junction_dist(self.lm, corr, s_ego, look)
                    if jd is not None:
                        road_ok, why = False, f"{jd:.0f} m sonra kavşak / şerit sonu (dönüş payı {look:.0f} m)"
                chain = self.chain_ahead(robjs, v_des)
                chain_short = False
                if chain is not None:
                    # Ardışık sollama ancak sol şerit sağdaki bu aracı da geçip önüne güvenli boşlukla dönmeye yetiyorsa.
                    # Yetmiyorsa solda kalmak, şerit bitiminde sıkışık zorunlu dönüşe yol açar (planning_validator
                    # "rear objects" çarpışma riski): o zaman hemen, bu aracın ARKASINA dön.
                    feas, why_c = self.chain_feasible(corr, s_ego, v_des, chain)
                    if not feas:
                        if self.chain_logged != ("kısa", chain["id"]):
                            self.chain_logged = ("kısa", chain["id"])
                            self.get_logger().info(f"ardışık sollama yok: sağ şeritte {chain['gap_front']:.0f} m önde "
                                                   f"{chain['v'] * 3.6:.0f} km/sa araç var ama {why_c}; onun arkasına dönülecek")
                        chain, chain_short = None, True
                        self.chain_seen_t = -1e9
                # Sağa dönüş: normalde MOBIL kazancı + boşluk; ardışık sollamaya yol yetmiyorsa yalnız güvenlik
                # (arkadaki araçla boşluk, yanımızda araç yok, öndeki yavaş araca rahat frenle yetişilebilir).
                ret_ok = (r["ok"] or (chain_short and r["safe"] and self.safe_behind(v, Lt))) and gap_ok and not ov
                mobil_ok = self.hold(ret_ok, t, M["hold_ret_s"])
                if chain:
                    self.chain_seen_t = t
                chain_recent = t - self.chain_seen_t < P["chain_clear_s"]   # algı kesintilerine karşı histerezis
                mobil_ok = mobil_ok and not chain_recent
                timeout = t - self.t_state > (P["max_left_chain_s"] if chain_recent else P["max_left_s"])
                if chain and road_ok and not timeout:
                    if self.chain_logged != chain["id"]:
                        self.chain_logged = chain["id"]
                        self.get_logger().info(f"ardışık sollama: sağ şeritte {chain['gap_front']:.0f} m önde {chain['v'] * 3.6:.0f} km/sa araç var, solda kalınıyor")
                    return
                if not (mobil_ok or timeout or not road_ok):
                    return
                reason = ((f"ardışık sollamaya yol yetmiyor, öndeki yavaş aracın arkasına, {self.fmt(r)}" if chain_short
                           else f"geçilen araç arkada, {self.fmt(r)}") if mobil_ok else ("sol şeritte süre doldu" if timeout else why))
                self.want_side("right", orig_side, reason)
                self.return_kind = "sollama"
                self.return_forced = not mobil_ok      # yol bitiyor / kavşak / süre doldu: ardışık sollama bunu iptal edemez
                self.goto("SAGA_DONUS", reason)
                return

            if self.state == "SAGA_DONUS":
                c_now = self.chain_ahead(robjs, v_des)
                if c_now is None:
                    self.chain_since = None
                elif self.chain_since is None:
                    self.chain_since = t
                if (self.return_kind == "sollama" and not self.return_forced and in_left and abs(corr.project(x, y)[1]) < 0.5
                        and c_now is not None and t - self.chain_since >= P["chain_confirm_s"] and t - self.t_state < P["max_left_chain_s"]
                        and self.chain_feasible(corr, s_ego, v_des, c_now)[0]):
                    c = self.chain_ahead(robjs, v_des)
                    self.want_side("left", orig_side, "ardışık sollama")
                    self.goto("GECIS", f"ardışık sollama: dönüş iptal, sağ şeritte {c['gap_front']:.0f} m önde {c['v'] * 3.6:.0f} km/sa araç")
                    return
                # Dönüş henüz başlamadıysa ve rota keep_right_release içinde sol şeridi gerektiriyorsa dönüşten vazgeç
                # (başlamış şerit değişimini kesmek planlamada ani kayma / MRM yapar: o zaman bitmesi beklenir).
                if in_left and orig_side == "left" and right_id is not None:
                    rc = self.lm.corridor(right_id, 0.0, P["keep_right_release"] + 20.0, self.route_ids)
                    lcs = self.lc_state("right")
                    started = abs(corr.project(x, y)[1]) > 0.5 or (lcs is not None and lcs[0] == "YURUTULUYOR")
                    if not started and not self.lane_in_route_ahead(rc, rc.project(x, y)[0], P["keep_right_release"]):
                        self.want_lane(2, f"sağa dönüş iptal: rota {P['keep_right_release']:.0f} m içinde sol şeridi gerektiriyor")
                        self.cooldown_until = t + P["cooldown_s"]
                        self.goto("TAKIP", f"sağa dönüş iptal: sağ şerit {P['keep_right_release']:.0f} m içinde rotadan ayrılıyor")
                        return
                if in_right or right_id is None:                  # sağ şeritteyiz (ya da tek şerit)
                    self.cooldown_until = t + P["cooldown_ok_s"]
                    if self.return_kind == "sollama":
                        self.stats["completed"] += 1
                        msg = "sollama tamam"
                    else:
                        self.stats["keep_right"] += 1
                        msg = "sağdan git: sağ şeride geçildi"
                    self.goto("TAKIP", f"{msg} (sollama başlayan {self.stats['started']}, biten {self.stats['completed']}, "
                                       f"vazgeçilen {self.stats['aborted']}; sağa geçiş {self.stats['keep_right']})")
                elif rejected and t - self.t_state > 10.0:      # geçici retler (şerit değişimi sürerken) yeniden denenir
                    self.rejected_right_ll = ego_ll
                    self.want_lane(2, "Autoware sağa kaydırmayı reddetti")
                    self.cooldown_until = t + P["cooldown_s"]
                    self.goto("TAKIP", "sağa geçiş reddedildi")
                elif self.return_kind != "sollama" and t - self.t_state > 30.0:
                    self.want_lane(2, "sağa geçiş 30 s içinde olmadı")
                    self.cooldown_until = t + P["cooldown_s"]
                    self.goto("TAKIP", "sağa geçişten vazgeçildi (zaman aşımı)")

        def chain_feasible(self, corr, s_ego, v_des, c):
            """Ardışık sollama: sol şerit, sağdaki yavaş c aracını da geçip önüne güvenli boşlukla dönmeye yetiyor mu?
            (can_overtake'in sollama başlangıcındaki mesafe hesabının aynısı; dönüş boşluğu return_min_gap/headway.)"""
            dv = max(v_des - c["v"], 0.5)
            ret_gap = max(P["return_min_gap"], P["return_min_headway"] * c["v"])
            pass_rel = c["gap_front"] + c["L"] + P["ego_front"] + P["ego_rear"] + ret_gap
            need = pass_rel / dv * v_des + P["lc_dist"]
            ok, why = road_clear(self.lm, corr, s_ego, need, side="right")
            if not ok:
                return ok, f"{need:.0f} m içinde {why}"
            jd = self.junction_ahead(need + junction_reserve(v_des))
            if jd is not None:
                return False, f"{jd:.0f} m sonra kavşak / şerit sonu; ardışık sollama + dönüş payı {need + junction_reserve(v_des):.0f} m"
            return True, ""

        def junction_ahead(self, dist):
            """Egonun şeridinde (rota boyunca) dist içinde kavşak / şerit sonu varsa uzaklığı; yoksa None."""
            if self.ego is None or self.ego_ll is None:
                return None
            x, y = self.ego[0], self.ego[1]
            jc = self.lm.corridor(self.ego_ll, 0.0, dist + 40.0, self.route_ids)
            s = jc.project(x, y)[0]
            return junction_dist(self.lm, jc, s, dist)

        @staticmethod
        def safe_behind(v, lead):
            """Sağ şeritteki öndeki yavaş aracın arkasına dönersek, ona rahat bir frenle (en çok b_safe) hızını eşitleyip
            en az return_min_gap boşlukla arkasında kalabilir miyiz? (Kinematik: IDM yaklaşma frenini abartır.)"""
            if lead is None:
                return True
            room = lead["gap_front"] - P["return_min_gap"]
            if room <= 0.0:
                return False
            dv = v - lead["v"]
            return dv <= 0.0 or dv * dv / (2.0 * room) <= M["b_safe"]

        def chain_ahead(self, robjs, v_des):
            """Sol şeritteyken sağ şeritte önümüzde (chain_dist içinde) geçilmeye değer yavaş araç var mı?"""
            c = [o for o in robjs if 0.0 < o["gap_front"] < P["chain_dist"] and P["lead_min_speed"] < o["v"] < v_des - P["min_speed_gap"]]
            return min(c, key=lambda o: o["gap_front"]) if c else None

        def lane_in_route_ahead(self, corr, s, dist):
            """Bu şerit s'den dist kadar ileriye rotanın parçası olarak sürüyor mu (ya da hedefe varıyor mu)?"""
            for i, a, b in corr.ranges:
                if b < s:
                    continue
                if a > s + dist:
                    break
                if i not in self.route_ids:
                    return False
                if i == self.goal_lanelet:
                    return True
            return corr.total - s >= dist

        def keep_right(self, t, x, y, v, v_des, Lc, Fo, robjs, ego_ll, right_id, orig_side):
            """Sol şeritteyiz ve sollama yapmıyoruz: sağ şerit boşsa sağa geç (asimetrik MOBIL'in sağa eğilimi).
            Asıl rota sağ şeridi tercih ediyorsa Autoware zaten sağa geçer; burada yalnızca asıl tercihin sol şerit
            olduğu ama sağ şeridin de rotada sürdüğü durumlar ele alınır."""
            if orig_side != "left" or right_id not in self.route_ids or ego_ll == self.rejected_right_ll:
                self.ok_since = None
                return
            if t < self.cooldown_until or self.pending or self.desired != self.confirmed:
                return
            rc = self.lm.corridor(right_id, 0.0, P["keep_right_need"] + 20.0, self.route_ids)
            s_r = rc.project(x, y)[0]
            if not self.lane_in_route_ahead(rc, s_r, P["keep_right_need"]):
                self.ok_since = None
                return                                          # rota yakında sol şeridi gerektiriyor: solda kal
            lc_ok, _ = road_clear(self.lm, rc, s_r, P["lc_dist"] + 20.0, side="left")
            Lt, Fn, ov = split_lane(robjs)
            npl = Lc if (Lc is not None and Lc["v"] < v - 0.5) else None
            r = mobil(v, v_des, (Lc, Fo), (Lt, Fn), to_left=False, no_pass_lead=npl)
            self.last_mobil = r
            gap_ok = Fn is None or Fn["gap_back"] >= max(P["return_min_gap"], P["return_min_headway"] * Fn["v"])
            if not self.hold(r["ok"] and gap_ok and not ov and lc_ok, t, M["hold_ret_s"]):
                return
            self.want_side("right", orig_side, f"sağdan git: sağ şerit boş ({self.fmt(r)})")
            self.return_kind = "sağdan git"
            self.goto("SAGA_DONUS", f"sağdan git: rota sol şeridi tercih ediyor ama sağ şerit boş ({self.fmt(r)})")

        def can_overtake(self, corr, s_ego, v_des, lead, left_id, lobjs, goal_dist):
            """Kural katmanı (MOBIL'in bilmediği şeyler): rota/harita, sollamanın tamamlanabilirliği, sol şeridin önde boş olması."""
            if self.ego_ll is not None and self.ego_ll == self.rejected_ll:
                return "Autoware bu lanelette sola kaydırmaya izin vermiyor (rota yapısı)"
            if left_id not in self.route_ids:
                return "sol şerit rotanın parçası değil"
            if goal_dist is not None and goal_dist < P["no_ot_before_goal"]:
                return "hedefe yakın"
            dv = max(v_des - lead["v"], 0.5)
            pass_rel = lead["gap_front"] + lead["L"] + P["ego_front"] + P["ego_rear"] + P["pass_margin_min"]
            need = pass_rel / dv * v_des + 2 * P["lc_dist"]
            ok, why = road_clear(self.lm, corr, s_ego, need)
            if not ok:
                return f"{need:.0f} m içinde {why}"
            res = junction_reserve(v_des)
            jd = self.junction_ahead(need + res)
            if jd is not None:
                return f"{jd:.0f} m sonra kavşak / şerit sonu; sollama {need:.0f} m + dönüş payı {res:.0f} m sığmıyor"
            for o in lobjs:
                if 0 < o["rel"] < lead["gap_front"] + P["left_free_extra"] and o["v"] < v_des - 1.0:
                    return f"sol şerit önde dolu ({o['rel']:.0f} m, {o['v'] * 3.6:.0f} km/sa)"
            return ""

        def shutdown(self):
            self.set_limit(None)
            if self.confirmed != 2 or self.desired != 2:
                self.set_lane(2, "kapanış")

    rclpy.init(args=sys.argv)
    node = OvertakeManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.shutdown()
        except Exception:
            pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    if len(sys.argv) >= 3 and sys.argv[1] == "--check":
        check_map(sys.argv[2])
    elif len(sys.argv) >= 2 and sys.argv[1] == "--mobil-test":
        mobil_selftest()
    else:
        run_node()
