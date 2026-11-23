"""Step "bulut": put the edited scene into the point-cloud map (needed for NDT localization).

The AWSIM build samples the scene surface inside the regions of the "awsim" step (median corridor + gate box)
and writes them to yollar.awsim_bulut. Here the base cloud's points inside those regions are replaced by them,
the result is written to yollar.bulut and cut into karo_m tiles (yollar.karolar, Autoware's dynamic map loading).
"""
import os, shutil
import numpy as np
from matplotlib.path import Path
from .ortak import yol, json_oku, yedekle, pcd_oku, pcd_yaz


def karola(xyz, hedef_klasor, G):
    """Write <hedef>/pointcloud_map/pointcloud_map_<i>_<j>.pcd + pointcloud_map_metadata.yaml (same as pcd_karola.py)."""
    ix = np.floor(xyz[:, 0] / G).astype(np.int64); iy = np.floor(xyz[:, 1] / G).astype(np.int64)
    o = np.lexsort((iy, ix)); xyz, ix, iy = xyz[o], ix[o], iy[o]
    key = ix * 1000003 + iy; st = np.r_[0, np.flatnonzero(np.diff(key)) + 1, len(key)]
    pdir = os.path.join(hedef_klasor, 'pointcloud_map')
    if os.path.isdir(pdir):
        shutil.rmtree(pdir)                      # a backup was taken by the caller
    os.makedirs(pdir)
    meta = [f'x_resolution: {G:.1f}', f'y_resolution: {G:.1f}']
    for a, b in zip(st[:-1], st[1:]):
        i, j = int(ix[a]), int(iy[a]); ad = f'pointcloud_map_{i}_{j}.pcd'
        pcd_yaz(os.path.join(pdir, ad), xyz[a:b])
        meta.append(f'{ad}: [{int(i * G)}, {int(j * G)}]')
    with open(os.path.join(hedef_klasor, 'pointcloud_map_metadata.yaml'), 'w') as f:
        f.write('\n'.join(meta) + '\n')
    return len(st) - 1


def _refuj_alti_sil(A, p):
    """Drop scene points under the median / island surfaces (old RoadRunner ground pushed below the curbs / grass): a real lidar
    cannot see them. Surface height = inverse-distance of the nearest curb / grass mesh vertices."""
    from scipy.spatial import cKDTree
    veri = json_oku(yol(A['yollar']['awsim_veri'])).get('refuj') or {}
    if not veri.get('sinir_xy'):
        return p
    halkalar = [veri['sinir_xy']] + [a['xy'] for a in veri.get('ek_sinirlar', [])]
    ic = np.zeros(len(p), bool)
    for h in halkalar:
        ic |= Path(np.array(h).reshape(-1, 2)).contains_points(p[:, :2])
    V = np.r_[np.array(veri['cim_v']).reshape(-1, 3), np.array(veri['kaldirim_v']).reshape(-1, 3),
              np.array(veri.get('asfalt_v', [])).reshape(-1, 3)]
    d, i = cKDTree(V[:, :2]).query(p[ic, :2], k=3); w = 1 / (d + 0.05)
    yuzey = (V[i, 2] * w).sum(1) / w.sum(1)
    alt = np.zeros(len(p), bool); alt[np.where(ic)[0][p[ic, 2] < yuzey - 0.04]] = True
    print(f'  refüj yüzeyinin altında kalan {alt.sum()} sahne noktası atıldı')
    return p[~alt]


def calistir(A):
    Y = A['yollar']; ara = yol(Y['ara'])
    alan_yolu = os.path.join(ara, 'bulut_alanlari.json')
    if not os.path.exists(alan_yolu):
        raise SystemExit('HATA: önce "awsim" adımını çalıştırın (bulut alanları yok)')
    bin_ = yol(Y['awsim_bulut'])
    if not os.path.exists(bin_):
        raise SystemExit('HATA: AWSIM bulutu yok: %s (AWSIM sahnesini derleyin)' % bin_)
    if os.path.getmtime(bin_) < os.path.getmtime(alan_yolu):
        print('  UYARI: AWSIM bulutu "awsim" adımından eski; sahne yeni veriyle derlenmemiş olabilir')
    print('bulut: temel bulut okunuyor (ilk sefer birkaç dakika sürebilir)')
    taban = pcd_oku(yol(Y['temel_bulut']), onbellek=os.path.join(ara, 'temel_bulut.npy'))[:, :3]
    yeni = np.fromfile(bin_, np.float32).reshape(-1, 3)
    icerde = np.zeros(len(taban), bool)
    for a in json_oku(alan_yolu):
        icerde |= Path(np.array(a['xy']).reshape(-1, 2)).contains_points(taban[:, :2])
    yeni = _refuj_alti_sil(A, yeni)
    m = np.r_[taban[~icerde], yeni].astype(np.float32)
    print(f'  temel {len(taban)} nokta, alanlarda çıkarılan {icerde.sum()}, AWSIM\'den eklenen {len(yeni)} -> {len(m)}')
    hedef = yol(Y['bulut']); yedekle(hedef, A); pcd_yaz(hedef, m)
    kd = yol(Y['karolar']); yedekle(os.path.join(kd, 'pointcloud_map'), A); yedekle(os.path.join(kd, 'pointcloud_map_metadata.yaml'), A)
    n = karola(m, kd, A['bulut']['karo_m'])
    print(f'bulut: {hedef} + {n} karo -> {kd}')
