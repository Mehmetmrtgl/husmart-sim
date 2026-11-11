"""Test sonuçlarından grafik + özet tablo üretir (host'ta çalışır).
Kullanım: python3 make_plots.py results/ map_graph.json out/"""
import json, os, sys, glob
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

res_dir, graph_json, out = sys.argv[1], sys.argv[2], sys.argv[3]
os.makedirs(out, exist_ok=True)
G = json.load(open(graph_json))["lanelets"]
results = [json.load(open(f)) for f in sorted(glob.glob(os.path.join(res_dir, "*.json")))]


def draw_map(ax, alpha=0.35):
    for v in G.values():
        xs = [v["start"][0], v["mid"][0], v["end"][0]]; ys = [v["start"][1], v["mid"][1], v["end"][1]]
        ax.plot(xs, ys, color="0.6", lw=0.6, alpha=alpha)


# 1) genel harita: tüm sürüş izleri
fig, ax = plt.subplots(figsize=(6, 11))
draw_map(ax)
for r in results:
    s = r.get("series") or {}
    if s.get("x"):
        ax.plot(s["x"], s["y"], lw=2.2, label=r["name"])
    if r.get("start"):
        ax.plot(*r["start"][:2], "o", ms=4, color="k")
        ax.plot(*r["goal"][:2], "*", ms=8, color="k")
ax.set_aspect("equal"); ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
ax.set_title("Beytepe v2 — test sürüş izleri"); ax.legend(fontsize=7, loc="upper left")
fig.tight_layout(); fig.savefig(os.path.join(out, "00_genel_harita.png"), dpi=130); plt.close(fig)

# 2) her test için: iz (yakın), hız, yanal sapma, ivme
for r in results:
    s = r.get("series") or {}
    if not s.get("t"):
        continue
    fig = plt.figure(figsize=(12, 7))
    ax0 = fig.add_subplot(1, 2, 1)
    draw_map(ax0, 0.6)
    ax0.plot(s["x"], s["y"], color="C0", lw=2.5)
    ax0.plot(*r["start"][:2], "go", ms=8, label="başlangıç"); ax0.plot(*r["goal"][:2], "r*", ms=12, label="hedef")
    pad = 30
    ax0.set_xlim(min(s["x"]) - pad, max(s["x"]) + pad); ax0.set_ylim(min(s["y"]) - pad, max(s["y"]) + pad)
    ax0.set_aspect("equal"); ax0.legend(fontsize=8); ax0.set_title(r.get("title", r["name"]))
    t = s["t"]
    ax1 = fig.add_subplot(3, 2, 2); ax1.plot(t, [v * 3.6 for v in s["v"]]); ax1.set_ylabel("hız [km/h]"); ax1.grid(alpha=.3)
    ax2 = fig.add_subplot(3, 2, 4); ax2.plot(t, s["dev"], color="C1"); ax2.set_ylabel("yanal sapma [m]"); ax2.grid(alpha=.3)
    ax3 = fig.add_subplot(3, 2, 6); ax3.plot(t, s["acc"], color="C2"); ax3.set_ylabel("ivme [m/s²]"); ax3.set_xlabel("zaman [s]"); ax3.grid(alpha=.3)
    for ev in r.get("mrm_events", []):
        for a in (ax1, ax2, ax3):
            a.axvline(ev[0], color="r", alpha=.3, lw=1)
    fig.suptitle(f"{r['name']} — {r['result']}: {r['reason']}", fontsize=10)
    fig.tight_layout(); fig.savefig(os.path.join(out, r["name"] + ".png"), dpi=110); plt.close(fig)

# 3) özet tablo (markdown)
cols = [("Test", "title"), ("Sonuç", "result"), ("Rota [m]", "route_length_m"), ("Süre [s]", "drive_time_s"),
        ("Maks hız [km/h]", "max_speed_kmh"), ("Maks sapma [m]", "lat_dev_max"), ("RMS sapma [m]", "lat_dev_rms"),
        ("Maks yavaşlama [m/s²]", "max_decel"), ("Hedefe uzaklık [m]", "final_goal_dist"), ("Açıklama", "reason")]
lines = ["| " + " | ".join(c[0] for c in cols) + " |", "|" + "---|" * len(cols)]
for r in results:
    lines.append("| " + " | ".join("" if r.get(k) is None else str(r.get(k)) for _, k in cols) + " |")
open(os.path.join(out, "ozet.md"), "w").write("\n".join(lines) + "\n")
print("\n".join(lines))
