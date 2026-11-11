#!/usr/bin/env python3
"""Yalıtılmış deney: tanılama grafiği 'hysteresis' ve 'warn-to-ok' birimleri kısa/uzun WARN darbelerine nasıl tepki veriyor.
Ayrı bir ROS alanında (ROS_DOMAIN_ID=98) çalışan test aggregator_node'una sahte /diagnostics yayınlar, /diagnostics_graph/status izler."""
import time, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rosidl_runtime_py.utilities import get_message

NAMES = ["plain", "hyst_leaf", "hyst_node", "w2o"]
# (başlangıç s, bitiş s, seviye): aradaki her 0.1 s'lik mesaj bu seviyede; geri kalan OK
PLAN = [(3.0, 3.1, 1), (5.0, 5.3, 1), (7.0, 7.6, 1), (10.0, 12.0, 1), (18.0, 18.1, 2), (20.0, 21.0, 2)]
ALT = (14.0, 16.0)   # her mesajda WARN/OK değişimi
END = 23.0

def level_at(t, k):
    if ALT[0] <= t < ALT[1]: return 1 if k % 2 == 0 else 0
    for a, b, lv in PLAN:
        if a <= t + 1e-6 < b: return lv
    return 0

rclpy.init(); n = Node("hyst_test")
pub = n.create_publisher(DiagnosticArray, "/diagnostics", 100)
for _ in range(100):
    types = dict(n.get_topic_names_and_types())
    ST = [k for k, v in types.items() if v[0].endswith("DiagGraphStruct")]; SS = [k for k, v in types.items() if v[0].endswith("DiagGraphStatus")]
    if ST and SS: break
    time.sleep(0.1)
print("graf konuları:", ST, SS)
S = get_message(types[ST[0]][0]); T = get_message(types[SS[0]][0])
paths = {}; last = {}; log = []; t0 = [None]
def on_struct(m):
    paths.clear()
    for i, nd in enumerate(m.nodes): paths[i] = nd.path
    print("struct:", sorted(paths.values()))
def on_status(m):
    if t0[0] is None: return
    t = time.monotonic() - t0[0]
    for i, nd in enumerate(m.nodes):
        p = paths.get(i)
        if p is None or not p.startswith("/t/"): continue
        lv = nd.level[0] if isinstance(nd.level, bytes) else nd.level
        if last.get(p) != lv:
            log.append((t, p, lv)); last[p] = lv
n.create_subscription(S, ST[0], on_struct, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
n.create_subscription(T, SS[0], on_status, 10)
t0[0] = time.monotonic(); k = 0; next_t = 0.0
while True:
    t = time.monotonic() - t0[0]
    if t >= END: break
    if t >= next_t:
        lv = level_at(next_t, k)
        arr = DiagnosticArray(); arr.header.stamp = n.get_clock().now().to_msg()
        for nm in NAMES:
            arr.status.append(DiagnosticStatus(level=bytes([lv]), name=f"test_node: {nm}", message=f"lv{lv}", hardware_id="test"))
        pub.publish(arr); k += 1; next_t += 0.1
    rclpy.spin_once(n, timeout_sec=0.01)
print("plan: WARN 0.1 s @3.0 | WARN 0.3 s @5.0 | WARN 0.6 s @7.0 | WARN 2.0 s @10.0 | WARN/OK değişken 14-16 | ERROR 0.1 s @18.0 | ERROR 1.0 s @20.0")
for p in sorted({x[1] for x in log}):
    print(p, " ".join(f"{t:5.2f}s:{lv}" for t, q, lv in log if q == p))
