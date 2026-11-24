#!/usr/bin/env python3
"""Beytepe map / simulation editing tool (gate road + median island).

Usage:  python3 tools/beytepe_duzenle/duzenle.py <step> [options]

Steps (in order):
  cizim         read the Google Earth drawing (veri/nizamiye_cizim.kml) -> uretilen/cizim_xy.json
  lanelet       gate lanes from the drawn routes -> harita/beytepe_yeni_v2/lanelet2_map.osm
  awsim         AWSIM scene data (gate road + median) -> AWSIM-src/Assets/Beytepe/Data/duzenlemeler.json
  kml           Google Earth check files (docs/ust_bakis/beytepe_sim.kml, uretilen/google_earth_hizali.kml)
  hepsi         cizim + lanelet + awsim + kml
  --- then build the AWSIM scene (Unity), it writes the scene point cloud ---
  bulut         put the AWSIM scene points into the point-cloud map + 50 m tiles
Needs the real-vehicle data (not in the repo):
  refuj-tespit  median lamps + trees from the real lidar map -> veri/refuj_nesneler.json
  refuj-guney-tespit  the same for the south part of the main road median -> veri/refuj_guney_nesneler.json
  kenar-tespit  trees beside the tour route from the real lidar map -> veri/yol_kenari_nesneler.json
  kontrol Y0 Y1 [--yon kuzey|guney] [--adet N]   draw the map onto real ZED camera frames

Every file that is overwritten is first copied to the backup folder (ayarlar.yaml: yollar.yedek).
"""
import argparse, os, sys, time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bd import ortak  # noqa: E402

ADIMLAR = ['cizim', 'lanelet', 'awsim', 'kml', 'hepsi', 'bulut', 'refuj-tespit', 'refuj-guney-tespit', 'kenar-tespit', 'kontrol']


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('adim', choices=ADIMLAR)
    ap.add_argument('y', nargs='*', type=float, help='kontrol: y0 y1')
    ap.add_argument('--yon', default='hepsi', choices=['hepsi', 'kuzey', 'guney'], help='kontrol: driving direction')
    ap.add_argument('--adet', type=int, default=8, help='kontrol: number of frames')
    ap.add_argument('--x', nargs=2, type=float, help='kontrol: x0 x1 (optional)')
    ap.add_argument('--ayarlar', help='settings file (default: ayarlar.yaml next to this script)')
    a = ap.parse_args()
    A = ortak.ayarlar(a.ayarlar)
    t0 = time.time()
    adimlar = ['cizim', 'lanelet', 'awsim', 'kml'] if a.adim == 'hepsi' else [a.adim]
    for adim in adimlar:
        if adim == 'cizim':
            from bd import cizim; cizim.calistir(A)
        elif adim == 'lanelet':
            from bd import nizamiye; nizamiye.calistir(A)
        elif adim == 'awsim':
            import subprocess
            if 'Unity -batchmode' in subprocess.run(['ps', '-eo', 'cmd'], capture_output=True, text=True).stdout:
                sys.exit('HATA: bir Unity (AWSIM) derlemesi çalışıyor; bitmesini bekleyin. Derleme sürerken veri değişirse sahneye eski veri girer.')
            try:
                import shapely  # noqa: F401
            except ImportError:
                sys.exit('HATA: "awsim" adımı shapely (>= 2.0) ister: pip install -r tools/beytepe_duzenle/requirements.txt')
            from bd import awsim; awsim.calistir(A)
        elif adim == 'kml':
            from bd import kml; kml.calistir(A)
        elif adim == 'bulut':
            from bd import bulut; bulut.calistir(A)
        elif adim == 'refuj-tespit':
            from bd import refuj; refuj.tespit_calistir(A)
        elif adim == 'refuj-guney-tespit':
            from bd import refuj; refuj.guney_tespit_calistir(A)
        elif adim == 'kenar-tespit':
            from bd import kenar; kenar.tespit_calistir(A)
        elif adim == 'kontrol':
            if len(a.y) != 2:
                ap.error('kontrol: y0 y1 gerekli, örnek: duzenle.py kontrol 2580 2680 --yon kuzey')
            from bd import kontrol
            kontrol.calistir(A, a.y[0], a.y[1], a.yon, a.adet, *(a.x or (None, None)))
    print('bitti (%.0f s)' % (time.time() - t0))
    if a.adim in ('awsim', 'hepsi'):
        print('sonraki adım: AWSIM sahnesini derleyin, sonra "duzenle.py bulut"')


if __name__ == '__main__':
    main()
