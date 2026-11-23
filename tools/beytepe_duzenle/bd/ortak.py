"""Shared helpers: settings, paths, backups, lanelet OSM reading, map-frame <-> lat/lon."""
import os, json, shutil, datetime
import xml.etree.ElementTree as ET
import numpy as np
import yaml

KOK = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))   # repo root (husmart-sim)


def ayarlar(yol=None):
    yol = yol or os.path.join(os.path.dirname(__file__), '..', 'ayarlar.yaml')
    with open(yol) as f:
        return yaml.safe_load(f)


def yol(p):
    p = os.path.expanduser(p)
    return p if os.path.isabs(p) else os.path.join(KOK, p)


def yedekle(dosya, A):
    """Copy an existing file/folder to the backup folder before it is overwritten."""
    if not os.path.exists(dosya):
        return None
    hedef = os.path.join(yol(A['yollar']['yedek']), datetime.datetime.now().strftime('%Y-%m-%d_%H%M%S'))
    os.makedirs(hedef, exist_ok=True)
    out = os.path.join(hedef, os.path.basename(dosya.rstrip('/')))
    (shutil.copytree if os.path.isdir(dosya) else shutil.copy2)(dosya, out)
    print('  yedek:', out)
    return out


def json_oku(p):
    with open(p) as f:
        return json.load(f)


def json_yaz(p, veri, A=None):
    if A is not None:
        yedekle(p, A)
    os.makedirs(os.path.dirname(p), exist_ok=True)
    with open(p, 'w') as f:
        json.dump(veri, f, ensure_ascii=False)


class Harita:
    """Lanelet2 OSM (local_x/local_y/ele tags). N: node -> (x,y,z); W: way -> [node]; WT: way tags; R: relation -> (members, tags)."""

    def __init__(self, p):
        self.yol = p
        self.metin = open(p).read()
        r = ET.fromstring(self.metin.encode())
        self.N, self.LL = {}, {}
        for n in r.iter('node'):
            t = {x.get('k'): x.get('v') for x in n.iter('tag')}
            if 'local_x' in t:
                self.N[n.get('id')] = np.array([float(t['local_x']), float(t['local_y']), float(t.get('ele', 0))])
                self.LL[n.get('id')] = (float(n.get('lat')), float(n.get('lon')))
        self.W = {w.get('id'): [nd.get('ref') for nd in w.iter('nd')] for w in r.iter('way')}
        self.WT = {w.get('id'): {x.get('k'): x.get('v') for x in w.iter('tag')} for w in r.iter('way')}
        self.R = {}
        for rel in r.iter('relation'):
            m = {x.get('role'): x.get('ref') for x in rel.iter('member') if x.get('role') in ('left', 'right')}
            self.R[rel.get('id')] = (m, {x.get('k'): x.get('v') for x in rel.iter('tag')})
        self.ids = [int(i) for i in list(self.N) + list(self.W) + list(self.R)]
        # speed bumps: regulatory element subtype speed_bump -> its polygon (x, y, z)
        self.kasisler = {}
        for rel in r.iter('relation'):
            if any(x.get('k') == 'subtype' and x.get('v') == 'speed_bump' for x in rel.iter('tag')):
                for m in rel.iter('member'):
                    if m.get('role') == 'refers' and m.get('ref') in self.W:
                        self.kasisler[m.get('ref')] = self.P(m.get('ref'))

    def P(self, wid):
        return np.array([self.N[i] for i in self.W[wid]])

    def serit(self, lid):
        m, _ = self.R[lid]
        return m['left'], m['right']

    def serit_alani(self, lid):
        l, r = self.serit(lid)
        return np.r_[self.P(l)[:, :2], self.P(r)[::-1, :2]]

    def yol_seritleri(self):
        return [lid for lid, (m, t) in self.R.items() if t.get('type') == 'lanelet' and t.get('subtype', 'road') == 'road' and 'left' in m]


class Izdusum:
    """Map frame (local_x, local_y) <-> WGS84, fitted on the map's own nodes (quadratic, ~mm accuracy)."""

    def __init__(self, harita):
        xy = np.array([harita.N[i][:2] for i in harita.N])
        ll = np.array([harita.LL[i] for i in harita.N])
        F = self._F(xy[:, 0], xy[:, 1])
        self.ia = np.linalg.lstsq(F, ll[:, 0], rcond=None)[0]
        self.io = np.linalg.lstsq(F, ll[:, 1], rcond=None)[0]
        self.la0, self.lo0 = ll[:, 0].mean(), ll[:, 1].mean()
        G = self._F((ll[:, 1] - self.lo0) * 1e4, (ll[:, 0] - self.la0) * 1e4)
        self.bx = np.linalg.lstsq(G, xy[:, 0], rcond=None)[0]
        self.by = np.linalg.lstsq(G, xy[:, 1], rcond=None)[0]

    @staticmethod
    def _F(x, y):
        x = np.asarray(x, float); y = np.asarray(y, float)
        return np.stack([x, y, x * x, y * y, x * y, np.ones_like(x)], -1)

    def xy2ll(self, x, y):
        F = self._F(x, y)
        return F @ self.ia, F @ self.io          # lat, lon

    def ll2xy(self, lon, lat):
        G = self._F((np.asarray(lon) - self.lo0) * 1e4, (np.asarray(lat) - self.la0) * 1e4)
        return np.c_[G @ self.bx, G @ self.by] if np.ndim(lon) else np.array([G @ self.bx, G @ self.by])


# ---------- PCD (x y z [intensity ...], float32; binary or binary_compressed) ----------
def _lzf(data, out_len):
    o = bytearray(out_len); ip = 0; op = 0; n = len(data)
    while ip < n:
        c = data[ip]; ip += 1
        if c < 32:
            c += 1; o[op:op + c] = data[ip:ip + c]; ip += c; op += c
        else:
            ln = c >> 5
            if ln == 7:
                ln += data[ip]; ip += 1
            ref = op - ((c & 0x1f) << 8) - data[ip] - 1; ip += 1; ln += 2
            if ref + ln <= op:
                o[op:op + ln] = o[ref:ref + ln]
            else:
                for k in range(ln):
                    o[op + k] = o[ref + k]
            op += ln
    if op != out_len:
        raise ValueError(f'LZF: {op} != {out_len}')
    return bytes(o)


def pcd_oku(p, onbellek=None):
    """Return an (n, fields) float32 array. binary_compressed files are slow to unpack in Python,
    so the result can be cached as .npy (rebuilt when the PCD is newer)."""
    if onbellek and os.path.exists(onbellek) and os.path.getmtime(onbellek) > os.path.getmtime(p):
        return np.load(onbellek)
    import struct
    raw = open(p, 'rb').read()
    he = raw.index(b'\nDATA ') + 1; le = raw.index(b'\n', he)
    h = {}
    for line in raw[:he].decode().splitlines():
        if line and not line.startswith('#'):
            k, *v = line.split(); h[k] = v
    nf = len(h['FIELDS']); n = int(h['POINTS'][0]); kind = raw[he:le].decode().split()[1]
    assert all(s == '4' for s in h['SIZE']) and all(t == 'F' for t in h['TYPE']), p
    body = raw[le + 1:]
    if kind == 'binary_compressed':
        cs, us = struct.unpack('<II', body[:8])
        a = np.frombuffer(_lzf(body[8:8 + cs], us), '<f4').reshape(nf, n).T.copy()
    elif kind == 'binary':
        a = np.frombuffer(body[:n * 4 * nf], '<f4').reshape(n, nf).copy()
    else:
        raise SystemExit('desteklenmeyen PCD: ' + kind)
    if onbellek:
        os.makedirs(os.path.dirname(onbellek), exist_ok=True); np.save(onbellek, a)
    return a


def pcd_yaz(p, xyz):
    xyz = np.ascontiguousarray(xyz[:, :3], '<f4')
    hd = ('# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
          f'WIDTH {len(xyz)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(xyz)}\nDATA binary\n')
    with open(p, 'wb') as f:
        f.write(hd.encode()); f.write(xyz.tobytes())
