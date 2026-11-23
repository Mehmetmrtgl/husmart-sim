"""Step "kontrol": draw the simulation map onto real ZED camera frames (needs the real-vehicle data).

Picks frames where the real vehicle was between y0 and y1 (optionally only driving north / south),
extracts them from the ZED video with ffmpeg inside the docker container, and draws
  lane borders (yellow = gate lanes, white = other lanes; colours below are RGB), median lamps (magenta), trees (green), bus shelter (cyan)
on the ground of the real lidar map. If the lines sit on the real road / the poles on the real poles, the map is right.
Output: uretilen/kontrol/<time>/ (frames) and a contact sheet uretilen/kontrol/kontrol_<y0>_<y1>_<yon>.jpg
"""
import os, subprocess, datetime
import numpy as np
from scipy.spatial.transform import Rotation as Rr, Slerp
from .ortak import Harita, yol, json_oku, pcd_oku


class Kamera:
    def __init__(self, A):
        K = A['kamera']; Y = A['yollar']
        d = np.load(yol(Y['gercek_poz'])); self.X, self.t = d['X'], d['t']
        self.zt = np.load(yol(Y['zed_zaman'])) * 1e-9
        T_il = np.eye(4); T_il[:3, :3] = Rr.from_euler('xyz', K['T_il_euler_xyz']).as_matrix(); T_il[:3, 3] = K['T_il_t']
        self.T_il_inv = np.linalg.inv(T_il); self.T_ci_inv = np.linalg.inv(np.array(K['T_ci']))
        self.f = K['K']; self.D = K['D']; self.dt = K['zaman_kaymasi_s']; self.w = K['genislik']

    def poz(self, tc):
        """camera -> map transform at camera time tc"""
        ti = tc + self.dt; i = np.clip(np.searchsorted(self.t, ti), 1, len(self.t) - 1)
        a = (ti - self.t[i - 1]) / (self.t[i] - self.t[i - 1]); P, Q = self.X[i - 1], self.X[i]
        T = np.eye(4); T[:3, :3] = Slerp([0, 1], Rr.from_matrix([P[:3, :3], Q[:3, :3]]))(a).as_matrix(); T[:3, 3] = P[:3, 3] * (1 - a) + Q[:3, 3] * a
        return T @ self.T_il_inv @ self.T_ci_inv

    def izdus(self, Pc):
        x = Pc[:, 0] / Pc[:, 2]; y = Pc[:, 1] / Pc[:, 2]; r2 = np.minimum(x * x + y * y, 1.5); k = 1 + self.D[0] * r2 + self.D[1] * r2 * r2
        return np.c_[self.f[0] * x * k + self.f[2], self.f[1] * y * k + self.f[3]]


def _zemin(A, lo, hi):
    """1 m ground grid (10th percentile of the real lidar), holes filled with the nearest cell."""
    from scipy import ndimage
    p = pcd_oku(yol(A['yollar']['gercek_lidar']))
    p = p[(p[:, 0] > lo[0]) & (p[:, 0] < hi[0]) & (p[:, 1] > lo[1]) & (p[:, 1] < hi[1]), :3]
    nx, ny = int(hi[0] - lo[0]) + 1, int(hi[1] - lo[1]) + 1
    ix = (p[:, 0] - lo[0]).astype(int); iy = (p[:, 1] - lo[1]).astype(int); key = iy * nx + ix
    o = np.argsort(key); ks = key[o]; z = p[o, 2]; u, st = np.unique(ks, return_index=True); en = np.r_[st[1:], len(ks)]
    G = np.full(nx * ny, np.nan)
    for k, a, b in zip(u, st, en):
        if b - a >= 3:
            G[k] = np.percentile(z[a:b], 10)
    G = G.reshape(ny, nx)
    # cells that only saw tree crowns / roofs: replace by the lowest ground around them
    Gf = np.where(np.isnan(G), 1e9, G); Gm = ndimage.minimum_filter(Gf, size=7)
    G = np.where(np.isnan(G) | (G - Gm > 0.3), Gm, G); G[G > 1e8] = np.nan
    idx = ndimage.distance_transform_edt(np.isnan(G), return_distances=False, return_indices=True)
    G = ndimage.median_filter(G[idx[0], idx[1]], size=3)
    return lambda x, y: G[np.clip((np.asarray(y) - lo[1]).astype(int), 0, ny - 1), np.clip((np.asarray(x) - lo[0]).astype(int), 0, nx - 1)]


def calistir(A, y0, y1, yon='hepsi', adet=8, x0=None, x1=None):
    import cv2
    Y = A['yollar']; kam = Kamera(A); p = kam.X[:, :2, 3]; v = np.gradient(p, axis=0)
    m = (p[:, 1] >= y0) & (p[:, 1] <= y1)
    if x0 is not None:
        m &= (p[:, 0] >= x0) & (p[:, 0] <= x1)
    if yon == 'kuzey':
        m &= v[:, 1] > 0.05
    elif yon == 'guney':
        m &= v[:, 1] < -0.05
    k = np.where(m)[0]
    if len(k) == 0:
        raise SystemExit('HATA: gerçek sürüşte bu aralıkta (%s) kayıt yok' % yon)
    k = k[np.unique(np.linspace(0, len(k) - 1, min(adet, len(k))).round().astype(int))]
    kare = sorted({int(np.argmin(abs(kam.zt - kam.t[i]))) for i in k})
    print(f'kontrol: {len(kare)} kare seçildi (y {y0}..{y1}, {yon})')
    # ---- extract the frames inside the container (ffmpeg is there), copy out, clean up
    zaman = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    yerel = os.path.join(yol(Y['ara']), 'kontrol', zaman); os.makedirs(yerel)
    ic = '/tmp/bd_kontrol_' + zaman; sec = '+'.join('eq(n\\,%d)' % i for i in kare)
    kon = Y['zed_konteyner']
    if kon not in subprocess.run(['docker', 'ps', '--format', '{{.Names}}'], capture_output=True, text=True).stdout.split():
        raise SystemExit('HATA: %s konteyneri çalışmıyor (docker start %s)' % (kon, kon))
    kmt = f'mkdir -p {ic} && ffmpeg -loglevel error -i {Y["zed_konteyner_video"]} -vf "select=\'{sec}\'" -vsync 0 -frames:v {len(kare)} {ic}/k_%02d.png'
    subprocess.run(['docker', 'exec', kon, 'bash', '-c', kmt], check=True)
    subprocess.run(['docker', 'cp', f'{kon}:{ic}/.', yerel], check=True)
    subprocess.run(['docker', 'exec', kon, 'rm', '-rf', ic], check=True)
    # ---- what to draw
    cam_xy = np.array([kam.poz(kam.zt[i])[:2, 3] for i in kare]); lo = cam_xy.min(0) - 90; hi = cam_xy.max(0) + 90
    zg = _zemin(A, lo, hi)
    H = Harita(yol(Y['harita'])); notu = A['nizamiye']['not']; cizgiler = []
    yeni_yol = {w for lid, (mm, t) in H.R.items() if notu in t.get('note', '') for w in mm.values()}
    for lid, (mm, t) in H.R.items():
        if t.get('type') != 'lanelet' or t.get('subtype', 'road') != 'road' or 'left' not in mm:
            continue
        for w in (mm['left'], mm['right']):
            P = H.P(w)[:, :2]
            if not ((P > lo).all(1) & (P < hi).all(1)).any():
                continue
            s = np.r_[0, np.cumsum(np.hypot(*np.diff(P, axis=0).T))]; ss = np.arange(0, s[-1] + 0.01, 0.5)
            q = np.c_[np.interp(ss, s, P[:, 0]), np.interp(ss, s, P[:, 1])]
            cizgiler.append((np.c_[q, zg(q[:, 0], q[:, 1]) + 0.03], (255, 220, 0) if w in yeni_yol else (255, 255, 255), 2 if w in yeni_yol else 1))
    direkler = []
    rp = os.path.join(yol(Y['ara']), 'refuj_ozet.json')
    if os.path.exists(rp):
        r = json_oku(rp)
        direkler += [(l['x'], l['y'], l['boy'], (255, 0, 255), l['ad']) for l in r['lambalar']]
        direkler += [(t['x'], t['y'], t['boy'], (0, 200, 0), '') for t in r['agaclar']]
    dp = os.path.join(yol(Y['ara']), 'durak.json')
    if os.path.exists(dp):
        d = json_oku(dp); direkler.append((d['x'], d['y'], 2.6, (0, 255, 255), 'durak'))
    # gate objects (veri/nizamiye_nesneler.json): poles + wall / guardrail / fence drawn at their top height
    np_ = yol(Y.get('nizamiye_nesneler', ''))
    if np_ and os.path.exists(np_):
        v = json_oku(np_)
        direkler += [(o['x'], o['y'], 1.0 + o.get('kol_boy', 3.8), (255, 140, 0), 'bariyer') for o in v.get('bariyerler', [])]
        direkler += [(o['x'], o['y'], o['boy'], (255, 255, 255), 'bayrak') for o in v.get('bayraklar', [])]
        direkler += [(o['x'], o['y'], o.get('boy', 3.0), (255, 60, 60), 'kulube') for o in v.get('kulubeler', [])]
        direkler += [(o['x'], o['y'], o.get('ust', 2.5), (60, 120, 255), 'tabela') for o in v.get('tabelalar', [])]
        direkler += [(o['x'], o['y'], o.get('boy', 1.5), (255, 220, 0), o['ad']) for o in v.get('kutular', [])]
        direkler += [(o['x'], o['y'], o.get('boy', 5.3), (255, 0, 0), 'pano') for o in v.get('panolar', [])]
        direkler += [(o['x'], o['y'], o.get('boy', 1.0), (0, 0, 255), 'cop') for o in v.get('cop_kutulari', [])]
        for o in v.get('buyuk_kulubeler', []):   # body outline at the ground and at the soffit, roof edge at the fascia top
            t_ = np.radians(o['aci_deg']); U_ = np.array([np.cos(t_), np.sin(t_)]); V_ = np.array([-np.sin(t_), np.cos(t_)]); C_ = np.array([o['x'], o['y']])
            for r, vs, z_list, renk in ((o['yari_en'], o['guney_v'], (0.05, o['tavan']), (255, 120, 0)),
                                        (o['yari_en'] + o['tasma'], o['guney_v'] - o['tasma_guney'], (o['tavan'], o['sacak_ust']), (0, 200, 120))):
                uv = [(r, vs)] + [(r * np.cos(a_), o['yay_v'] + r * np.sin(a_)) for a_ in np.linspace(0, np.pi, 25)] + [(-r, vs), (r, vs)]
                q = np.array([C_ + U_ * u + V_ * w for u, w in uv]); zb = float(zg(*C_))
                for z_ in z_list:
                    cizgiler.append((np.c_[q, np.full(len(q), zb + z_)], renk, 2))
        for k, renk in (('duvarlar', (255, 0, 255)), ('korkuluklar', (0, 255, 255)), ('citler', (0, 255, 0))):
            for o in v.get(k, []):
                P = np.array(o['xy']).reshape(-1, 2); h = o.get('boy', 0.77 if k == 'korkuluklar' else 2.0)
                hs = np.array(h, float) if isinstance(h, list) else np.full(len(P), float(h))
                s = np.r_[0, np.cumsum(np.hypot(*np.diff(P, axis=0).T))]; ss = np.arange(0, s[-1] + 0.01, 0.5)
                q = np.c_[np.interp(ss, s, P[:, 0]), np.interp(ss, s, P[:, 1])]; hq = np.interp(ss, s, hs)
                cizgiler.append((np.c_[q, zg(q[:, 0], q[:, 1]) + hq], renk, 2))
                cizgiler.append((np.c_[q, zg(q[:, 0], q[:, 1]) + 0.05], renk, 1))
    # ---- draw
    outs = []
    for j, fi in enumerate(kare, 1):
        im = cv2.imread(os.path.join(yerel, 'k_%02d.png' % j))
        if im is None:
            continue
        im = im[:, :kam.w].copy(); hh = im.shape[0]; Ti = np.linalg.inv(kam.poz(kam.zt[fi]))
        for Q, renk, kal in cizgiler:
            Pc = np.c_[Q, np.ones(len(Q))] @ Ti.T; ok = (Pc[:, 2] > 1.0) & (Pc[:, 2] < 80)
            uv = kam.izdus(Pc[:, :3]); ok &= (uv[:, 0] > -200) & (uv[:, 0] < kam.w + 200) & (uv[:, 1] > -200) & (uv[:, 1] < hh + 200)
            for a in range(len(Q) - 1):
                if ok[a] and ok[a + 1]:
                    cv2.line(im, tuple(uv[a].astype(int)), tuple(uv[a + 1].astype(int)), renk[::-1], kal, cv2.LINE_AA)
        for x, y, boy, renk, ad in direkler:
            z0 = float(zg(x, y)); Pc = np.array([[x, y, z0, 1], [x, y, z0 + boy, 1]]) @ Ti.T
            if Pc[0, 2] < 1.5 or Pc[0, 2] > 80:
                continue
            uv = kam.izdus(Pc[:, :3]).astype(int)
            if not (-50 < uv[0, 0] < kam.w + 50):
                continue
            cv2.line(im, tuple(uv[0]), tuple(uv[1]), renk[::-1], 2, cv2.LINE_AA)
            if ad:
                cv2.putText(im, ad, (uv[0, 0] - 10, uv[0, 1] + 20), 0, 0.6, renk[::-1], 2)
        cx, cy = kam.poz(kam.zt[fi])[:2, 3]
        cv2.putText(im, 'kare %d  x %.0f y %.0f' % (fi, cx, cy), (10, 30), 0, 0.8, (255, 255, 255), 2)
        cv2.imwrite(os.path.join(yerel, 'cizili_%02d.jpg' % j), im); outs.append(im)
    while len(outs) % 2:
        outs.append(np.zeros_like(outs[0]))
    sheet = np.vstack([np.hstack(outs[i:i + 2]) for i in range(0, len(outs), 2)])
    hedef = os.path.join(yol(Y['ara']), 'kontrol', f'kontrol_{y0:.0f}_{y1:.0f}_{yon}.jpg')
    cv2.imwrite(hedef, sheet, [cv2.IMWRITE_JPEG_QUALITY, 85])
    print(f'kontrol: {hedef}  (tek tek kareler: {yerel})')
    return hedef
