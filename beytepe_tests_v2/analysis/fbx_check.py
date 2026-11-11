"""Binary FBX (7.x) okuyucu + harita hizalama kontrolü.
FBX'teki tüm mesh köşelerini dünya koordinatına çevirir, eksen/birim ayarlarını raporlar ve
v2 PCD / lanelet ile karşılaştırarak Unity'de gereken dönüşümü bulmaya yardım eder.
Kullanım: python3 -I fbx_check.py <fbx> <pcd_sub.npy> <lanelet2_map.osm>
"""
import struct, sys, zlib, math, json
import numpy as np


# ------------------------------------------------------------------ FBX binary parser
def read_fbx(path):
    data = open(path, "rb").read()
    ver = struct.unpack_from("<I", data, 23)[0]
    big = ver >= 7500
    hdr = "<QQQ" if big else "<III"
    hsz = struct.calcsize(hdr)

    def read_prop(o):
        t = chr(data[o]); o += 1
        if t == "Y": return struct.unpack_from("<h", data, o)[0], o + 2
        if t == "C": return bool(data[o]), o + 1
        if t == "I": return struct.unpack_from("<i", data, o)[0], o + 4
        if t == "F": return struct.unpack_from("<f", data, o)[0], o + 4
        if t == "D": return struct.unpack_from("<d", data, o)[0], o + 8
        if t == "L": return struct.unpack_from("<q", data, o)[0], o + 8
        if t in "SR":
            n = struct.unpack_from("<I", data, o)[0]; o += 4
            b = data[o:o + n]
            return (b.decode("utf-8", "replace") if t == "S" else b), o + n
        if t in "fdlib":
            n, enc, clen = struct.unpack_from("<III", data, o); o += 12
            raw = data[o:o + clen]; o += clen
            if enc == 1:
                raw = zlib.decompress(raw)
            dt = {"f": "<f4", "d": "<f8", "l": "<i8", "i": "<i4", "b": "u1"}[t]
            return np.frombuffer(raw, dtype=dt, count=n), o
        raise ValueError(f"bilinmeyen özellik tipi {t!r} @ {o}")

    def read_node(o):
        end, nprops, plen = struct.unpack_from(hdr, data, o); o += hsz
        if end == 0:
            return None, o
        nlen = data[o]; o += 1
        name = data[o:o + nlen].decode("ascii", "replace"); o += nlen
        props = []
        for _ in range(nprops):
            v, o = read_prop(o); props.append(v)
        children = []
        while o < end:
            c, o2 = read_node(o)
            if c is None:
                o = o2; break
            children.append(c); o = o2
        return dict(name=name, props=props, children=children), end

    o = 27; top = []
    while o < len(data) - hsz:
        n, o2 = read_node(o)
        if n is None: break
        top.append(n); o = o2
    return ver, top


def child(n, name):
    for c in n["children"]:
        if c["name"] == name: return c
    return None


def props70(n):
    out = {}
    p = child(n, "Properties70")
    if p:
        for c in p["children"]:
            if c["name"] == "P":
                out[c["props"][0]] = c["props"][4:]
    return out


def euler_xyz(r):
    """FBX varsayılan döndürme sırası eXYZ (önce X): R = Rz*Ry*Rx"""
    rx, ry, rz = np.radians(r)
    cx, sx, cy, sy, cz, sz = math.cos(rx), math.sin(rx), math.cos(ry), math.sin(ry), math.cos(rz), math.sin(rz)
    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def local_matrix(p):
    T = np.eye(4)
    t = np.array(p.get("Lcl Translation", [0, 0, 0]), float)
    r = np.array(p.get("Lcl Rotation", [0, 0, 0]), float)
    s = np.array(p.get("Lcl Scaling", [1, 1, 1]), float)
    pre = np.array(p.get("PreRotation", [0, 0, 0]), float)
    R = euler_xyz(pre) @ euler_xyz(r)
    T[:3, :3] = R @ np.diag(s); T[:3, 3] = t
    return T


def geometric_matrix(p):
    T = np.eye(4)
    T[:3, :3] = euler_xyz(np.array(p.get("GeometricRotation", [0, 0, 0]), float)) @ np.diag(
        np.array(p.get("GeometricScaling", [1, 1, 1]), float))
    T[:3, 3] = np.array(p.get("GeometricTranslation", [0, 0, 0]), float)
    return T


def main():
    fbx, pcd_npy, osm = sys.argv[1:4]
    ver, top = read_fbx(fbx)
    root = {n["name"]: n for n in top}
    gs = props70(child(root["GlobalSettings"], "Properties70") and root["GlobalSettings"] or root["GlobalSettings"])
    axes = {k: gs.get(k, [None])[0] for k in ("UpAxis", "UpAxisSign", "FrontAxis", "FrontAxisSign",
                                              "CoordAxis", "CoordAxisSign", "UnitScaleFactor", "OriginalUnitScaleFactor")}
    objs = root["Objects"]["children"]
    geoms = {o["props"][0]: o for o in objs if o["name"] == "Geometry"}
    models = {o["props"][0]: o for o in objs if o["name"] == "Model"}
    parent = {}; geom_model = {}
    for c in root["Connections"]["children"]:
        if c["props"][0] != "OO": continue
        a, b = c["props"][1], c["props"][2]
        if a in models and (b in models or b == 0): parent[a] = b
        if a in geoms and b in models: geom_model[a] = b

    def world(mid):
        M = np.eye(4); x = mid
        while x in models:
            M = local_matrix(props70(models[x])) @ M
            x = parent.get(x, 0)
        return M

    used_pre = sum(1 for m in models.values() if "PreRotation" in props70(m))
    used_geo = sum(1 for m in models.values() if any(k.startswith("Geometric") for k in props70(m)))
    pts = []; names = []
    for gid, g in geoms.items():
        v = child(g, "Vertices")
        if v is None or gid not in geom_model: continue
        V = np.asarray(v["props"][0], float).reshape(-1, 3)
        mid = geom_model[gid]
        M = world(mid) @ geometric_matrix(props70(models[mid]))
        W = (np.c_[V, np.ones(len(V))] @ M.T)[:, :3]
        pts.append(W); names.append((models[mid]["props"][1].split("\x00")[0], len(V)))
    P = np.vstack(pts)
    usf = axes["UnitScaleFactor"] or 1.0
    Pm = P * usf / 100.0   # FBX birimi cm; UnitScaleFactor=100 -> metre
    report = dict(fbx_version=ver, axes=axes, n_models=len(models), n_geoms=len(geoms),
                  n_vertices=int(len(P)), models_with_prerotation=used_pre, models_with_geometric=used_geo,
                  bbox_m_min=Pm.min(0).round(2).tolist(), bbox_m_max=Pm.max(0).round(2).tolist(),
                  biggest_meshes=sorted(names, key=lambda x: -x[1])[:12])
    print(json.dumps(report, ensure_ascii=False, indent=1))

    # --- olası eksen eşleşmelerini PCD ile karşılaştır (harita çerçevesi: x=local_x, y=local_y, z=ele)
    pcd = np.load(pcd_npy)[:, :3]
    sub = Pm[np.random.default_rng(0).choice(len(Pm), min(60000, len(Pm)), replace=False)]
    cell = 1.0
    keys = {}
    for p in pcd:
        keys.setdefault((int(math.floor(p[0] / cell)), int(math.floor(p[1] / cell))), []).append(p[2])
    zmed = {k: float(np.median(v)) for k, v in keys.items()}
    cands = {}
    perms = [(0, 1, 2), (0, 2, 1), (1, 0, 2), (1, 2, 0), (2, 0, 1), (2, 1, 0)]
    for perm in perms:
        for sx in (1, -1):
            for sy in (1, -1):
                for sz in (1, -1):
                    Q = np.c_[sx * sub[:, perm[0]], sy * sub[:, perm[1]], sz * sub[:, perm[2]]]
                    hit = []
                    for q in Q:
                        k = (int(math.floor(q[0] / cell)), int(math.floor(q[1] / cell)))
                        if k in zmed: hit.append(q[2] - zmed[k])
                    if len(hit) > 200:
                        h = np.array(hit)
                        cands[f"x={'+-'[sx<0]}{'XYZ'[perm[0]]} y={'+-'[sy<0]}{'XYZ'[perm[1]]} z={'+-'[sz<0]}{'XYZ'[perm[2]]}"] = (
                            len(hit) / len(Q), float(np.median(h)), float(np.percentile(np.abs(h - np.median(h)), 50)))
    best = sorted(cands.items(), key=lambda kv: (-kv[1][0], kv[1][2]))[:6]
    print("\nEksen eşleşmesi adayları (FBX metre -> harita): kapsama oranı, medyan dz, |dz| medyan sapma")
    for k, (cov, dz, mad) in best:
        print(f"  {k:28s}  kapsama {cov:5.2f}  dz {dz:+8.2f} m  MAD {mad:6.2f} m")


if __name__ == "__main__":
    main()
