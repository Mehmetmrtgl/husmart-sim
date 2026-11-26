"""Road-surface repair: smooth the RoadRunner road mesh where it has steps / sharp creases that the real road does not have.

The AWSIM car jumped (~1 g vertical) at the K3 exit, the return-leg merge and inside K13 in every tour since the first one.
The RoadRunner model (Beytepe.fbx) has a 13 cm ramp over < 1 m, a 7 cm step under the right wheel track and 10 % grade
breaks there; the real drive recording (Xsens IMU) shows nothing above its noise level at the same places.

For every zone (circle, ayarlar.yaml `yol_duzelt.bolgeler`):
  * the top road surface (Roads_Road) is sampled from Beytepe.fbx on a grid;
  * inside the circle it is smoothed (thin-plate smoothing spline: least squares of the discrete Laplacian plus a weak pull
    towards the old surface, smoothing length `puruz_boyu_m`) while the surface around the circle stays fixed, so the edge
    joins without a step or a kink;
  * the height change (delta) is written to duzenlemeler.json; the scene builder (BeytepeDuzenleme.cs ApplyYolDuzelt)
    puts the road-layer vertices inside the circle on the new surface (hedef) and adds the change of the top surface (delta)
    to every other terrain layer there (marking, gutter, curb, sidewalk, grass), so curbs and gutters keep their shape
    relative to the road. Off the road the change fades out over `kenar_sonum_m`.
"""
import base64, struct, zlib
import numpy as np
from scipy import sparse
from scipy.sparse.linalg import spsolve
from scipy.spatial import cKDTree
from .ortak import yol


# ---------------- minimal binary FBX reader (geometry only) ----------------
def _fbx_dugumler(b):
    ver = struct.unpack_from('<I', b, 23)[0]; big = ver >= 7500

    def dugum(o):
        if big:
            end, nprop, _ = struct.unpack_from('<QQQ', b, o); o += 24
        else:
            end, nprop, _ = struct.unpack_from('<III', b, o); o += 12
        nl = b[o]; o += 1
        if end == 0:
            return None, o
        ad = b[o:o + nl].decode('latin1'); o += nl
        props = []
        for _ in range(nprop):
            t = chr(b[o]); o += 1
            if t in 'YCIFDL':
                fmt = {'Y': '<h', 'C': '<?', 'I': '<i', 'F': '<f', 'D': '<d', 'L': '<q'}[t]
                props.append(struct.unpack_from(fmt, b, o)[0]); o += struct.calcsize(fmt)
            elif t in 'fdlib':
                n, enc, cl = struct.unpack_from('<III', b, o); o += 12
                raw = b[o:o + cl]; o += cl
                if enc:
                    raw = zlib.decompress(raw)
                props.append(np.frombuffer(raw, dtype={'f': '<f4', 'd': '<f8', 'l': '<i8', 'i': '<i4', 'b': '?'}[t]))
            elif t in 'SR':
                n = struct.unpack_from('<I', b, o)[0]; o += 4
                props.append(b[o:o + n]); o += n
        kids = []
        while o < end:
            k, o2 = dugum(o)
            if k is None:
                break
            kids.append(k); o = o2
        return (ad, props, kids), end

    top, o = [], 27
    while o < len(b) - 200:
        n, o2 = dugum(o)
        if n is None:
            break
        top.append(n); o = o2
    return top


def fbx_ucgenler(path, model_adi):
    """(vertices, triangles) of the FBX geometry whose model name contains model_adi (FBX coordinates = ROS map x, y, z)."""
    b = open(path, 'rb').read()
    assert b[:20] == b'Kaydara FBX Binary  ', 'not a binary FBX: ' + path
    top = _fbx_dugumler(b)
    objs = [t for t in top if t[0] == 'Objects'][0][2]
    conns = [t for t in top if t[0] == 'Connections'][0][2]
    geo, models = {}, {}
    for o in objs:
        if o[0] == 'Geometry':
            V = [k for k in o[2] if k[0] == 'Vertices']; I = [k for k in o[2] if k[0] == 'PolygonVertexIndex']
            if V and I:
                geo[o[1][0]] = (V[0][1][0].reshape(-1, 3), I[0][1][0])
        elif o[0] == 'Model':
            models[o[1][0]] = o[1][1].split(b'\x00')[0].decode('latin1')
    for c in conns:
        if c[0] == 'C' and c[1][0] == b'OO' and c[1][1] in geo and models.get(c[1][2], '').find(model_adi) >= 0:
            V, I = geo[c[1][1]]
            tris, poly = [], []
            for k in I:
                if k < 0:
                    poly.append(~k); tris += [(poly[0], poly[j], poly[j + 1]) for j in range(1, len(poly) - 1)]; poly = []
                else:
                    poly.append(k)
            return V.astype(float), np.array(tris, dtype=np.int64)
    raise KeyError('FBX model not found: ' + model_adi)


def yuzey(V, F, px, py):
    """Highest surface z of the triangles (V, F) at points (px, py); nan where there is none."""
    A, B, C = V[F[:, 0]], V[F[:, 1]], V[F[:, 2]]
    cen = (A + B + C)[:, :2] / 3
    rad = np.max(np.stack([np.hypot(*(P[:, :2] - cen).T) for P in (A, B, C)]), 0)
    kd = cKDTree(cen); rmax = rad.max()
    z = np.full(len(px), np.nan)
    for i, (x, y) in enumerate(zip(px, py)):
        for t in kd.query_ball_point([x, y], rmax):
            if np.hypot(x - cen[t, 0], y - cen[t, 1]) > rad[t] + 1e-6:
                continue
            a, b, c = A[t], B[t], C[t]
            d = (b[1] - c[1]) * (a[0] - c[0]) + (c[0] - b[0]) * (a[1] - c[1])
            if abs(d) < 1e-12:
                continue
            l1 = ((b[1] - c[1]) * (x - c[0]) + (c[0] - b[0]) * (y - c[1])) / d
            l2 = ((c[1] - a[1]) * (x - c[0]) + (a[0] - c[0]) * (y - c[1])) / d
            if min(l1, l2, 1 - l1 - l2) < -1e-6:
                continue
            zz = l1 * a[2] + l2 * b[2] + (1 - l1 - l2) * c[2]
            if np.isnan(z[i]) or zz > z[i]:
                z[i] = zz
    return z


# ---------------- smoothest fill ----------------
def puruzsuz(Z, icerde, lam):
    """Smoothing of the road surface Z (nan = no road) at the icerde cells; every other road cell stays fixed.
    Minimises sum over road cells of (L z)^2 + lam * sum over icerde cells of (z - Z)^2 (L = 4-neighbour graph Laplacian).
    The data term keeps the large-scale shape (smoothing length ~ cell / lam^0.25); steps and creases shorter than that
    are spread out. Without it the fill drifted up to 40 cm along the road edges inside the circle."""
    ny, nx = Z.shape
    yol_h = ~np.isnan(Z)
    no = -np.ones(Z.shape, dtype=np.int64); no[yol_h] = np.arange(yol_h.sum())
    rows, cols, vals = [], [], []
    for j, i in zip(*np.nonzero(yol_h)):
        k = no[j, i]; nb = [no[j + dj, i + di] for dj, di in ((0, 1), (0, -1), (1, 0), (-1, 0))
                            if 0 <= j + dj < ny and 0 <= i + di < nx and no[j + dj, i + di] >= 0]
        rows += [k] * (len(nb) + 1); cols += [k] + nb; vals += [float(len(nb))] + [-1.0] * len(nb)
    n = int(yol_h.sum())
    L = sparse.csr_matrix((vals, (rows, cols)), shape=(n, n))
    z = Z[yol_h]
    u = icerde[yol_h]
    Lu, Lf = L[:, u], L[:, ~u]
    M = (Lu.T @ Lu + lam * sparse.identity(int(u.sum()))).tocsc()
    zu = spsolve(M, -(Lu.T @ (Lf @ z[~u])) + lam * z[u])
    out = Z.copy(); tmp = z.copy(); tmp[u] = zu; out[yol_h] = tmp
    return out


def delik_doldur(Z, en_cok=8, catlak=2):
    """Gaps in the sampled road surface get the mean of their road neighbours: small holes (<= en_cok cells) and thin
    cracks (closed by `catlak` cells; RoadRunner polygons leave T-junction cracks, and a crack cut the smoothing in two
    so the sides drifted apart by 11 cm). Larger empty areas (off the road) stay empty."""
    from scipy import ndimage
    bos = np.isnan(Z)
    lab, n = ndimage.label(bos)
    doldur = np.zeros_like(bos)
    if n:
        boy = ndimage.sum(bos, lab, index=np.arange(1, n + 1))
        doldur |= np.isin(lab, np.nonzero(boy <= en_cok)[0] + 1)
    doldur |= ndimage.binary_closing(~bos, structure=np.ones((3, 3)), iterations=catlak) & bos
    Z = Z.copy()
    for _ in range(2 * catlak + 4):
        eksik = doldur & np.isnan(Z)
        if not eksik.any():
            break
        P = np.pad(Z, 1, constant_values=np.nan)
        komsu = np.stack([P[1:-1, 2:], P[1:-1, :-2], P[2:, 1:-1], P[:-2, 1:-1]])
        say = (~np.isnan(komsu)).sum(0)
        ort = np.where(say > 0, np.nansum(komsu, 0) / np.maximum(say, 1), np.nan)
        Z[eksik & (say > 0)] = ort[eksik & (say > 0)]
    return Z


def _b64(a):
    return base64.b64encode(np.ascontiguousarray(a, dtype='<f4').tobytes()).decode()


def veri(A, yollar_xy=()):
    """duzenlemeler.json section + cloud regions. yollar_xy: optional list of (N, 2) driving paths for the wheel-track check."""
    Y = A.get('yol_duzelt') or {}
    bolgeler = Y.get('bolgeler') or []
    if not bolgeler:
        return None, []
    V, F = fbx_ucgenler(yol(A['yollar']['roadrunner_fbx']), 'Roads_Road')
    r = float(Y.get('cozunurluk_m', 0.25)); pay = float(Y.get('kenar_sonum_m', 3.0)); dis = float(Y.get('sinir_bant_m', 3.0))
    lam = (r / float(Y.get('puruz_boyu_m', 2.5))) ** 4
    out, alanlar = [], []
    for b in bolgeler:
        cx, cy = map(float, b['merkez']); R = float(b['yaricap_m'])
        H = R + dis + pay
        xs = np.arange(cx - H, cx + H + 1e-6, r); ys = np.arange(cy - H, cy + H + 1e-6, r)
        X, Yg = np.meshgrid(xs, ys)
        Z = delik_doldur(yuzey(V, F, X.ravel(), Yg.ravel()).reshape(X.shape))
        rr = np.hypot(X - cx, Yg - cy)
        Z[rr > R + dis] = np.nan                    # the fixed band around the circle is the boundary condition
        icerde = (rr <= R) & ~np.isnan(Z)
        Zn = puruzsuz(Z, icerde, lam)
        D = np.where(icerde, Zn - Z, 0.0)
        D[np.isnan(D)] = 0.0
        Hd = np.where(icerde, Zn, -999.0)          # road-layer vertices inside the circle are put on this surface
        # off the road (grass, curb tops without asphalt below) the change of the nearest road cell fades out over `pay`
        yol_h = ~np.isnan(Z) & (rr <= R + dis)
        kd = cKDTree(np.c_[X[yol_h], Yg[yol_h]])
        dd, ii = kd.query(np.c_[X.ravel(), Yg.ravel()])
        Dext = D[yol_h][ii] * np.clip(1 - dd / pay, 0, 1)
        D = np.where(yol_h, D, Dext.reshape(D.shape))
        # report: steepest grade change along the wheel tracks before / after
        rapor = ''
        for P in yollar_xy:
            P = np.asarray(P, float); w = np.hypot(P[:, 0] - cx, P[:, 1] - cy) < R + 2
            if w.sum() < 3:
                continue
            s = np.r_[0, np.cumsum(np.hypot(*np.diff(P[w], axis=0).T))]
            ss = np.arange(0, s[-1], 0.1); qx = np.interp(ss, s, P[w, 0]); qy = np.interp(ss, s, P[w, 1])
            hd = np.arctan2(np.gradient(qy), np.gradient(qx))
            for off in (-0.8, 0.8):
                tx, ty = qx - np.sin(hd) * off, qy + np.cos(hd) * off
                z0 = yuzey(V, F, tx, ty)
                fi = (tx - xs[0]) / r; fj = (ty - ys[0]) / r
                i0 = np.clip(np.floor(fi).astype(int), 0, len(xs) - 2); j0 = np.clip(np.floor(fj).astype(int), 0, len(ys) - 2)
                u, v = np.clip(fi - i0, 0, 1), np.clip(fj - j0, 0, 1)
                c4 = [Hd[j0, i0], Hd[j0, i0 + 1], Hd[j0 + 1, i0], Hd[j0 + 1, i0 + 1]]
                hz = c4[0] * (1 - u) * (1 - v) + c4[1] * u * (1 - v) + c4[2] * (1 - u) * v + c4[3] * u * v
                dz = np.where(np.min(c4, 0) > -900, hz - z0, 0.0)     # as in the scene builder: snapped inside, unchanged outside
                k = 3
                ic = np.hypot(tx - cx, ty - cy)[k:-k] <= R + dis
                def kirilma(z):
                    g1 = (z[k:-k] - z[:-2 * k]) / (k * 0.1); g2 = (z[2 * k:] - z[k:-k]) / (k * 0.1)
                    a = np.where(ic, np.abs(g2 - g1), np.nan); j = int(np.nanargmax(a))
                    return a[j] * 100, np.hypot(tx[j + k] - cx, ty[j + k] - cy)
                (k0, d0), (k1, d1) = kirilma(z0), kirilma(z0 + dz)
                rapor += ' teker %+.1f: %%%.0f (%.0f m) -> %%%.0f (%.0f m);' % (off, k0, d0, k1, d1)
        print('  yol düzeltme %s: R %.1f m, en büyük değişim %+.1f / %+.1f cm;%s' % (b['ad'], R, D.min() * 100, D.max() * 100, rapor))
        out.append(dict(ad=b['ad'], x0=round(float(xs[0] - r / 2), 3), y0=round(float(ys[0] - r / 2), 3), cozunurluk=r,
                        nx=len(xs), ny=len(ys), hedef=_b64(Hd), delta=_b64(D)))
        ang = np.linspace(0, 2 * np.pi, 33)[:-1]
        alanlar.append(dict(xy=[round(float(c), 2) for a in ang for c in (cx + (H + 1) * np.cos(a), cy + (H + 1) * np.sin(a))]))
    return dict(bolgeler=out), alanlar
