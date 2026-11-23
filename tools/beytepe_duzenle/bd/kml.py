"""Step "kml": Google Earth files to check the simulation map from above.

yollar.sim_kml (docs/ust_bakis/beytepe_sim.kml): buildings (kept from the existing file) + all lane lines
    + the gate lanes (named centre lines) + the median (outline, lamps) + bus shelter, in the MAP frame.
uretilen/google_earth_hizali.kml: only the gate lanes and the median, shifted by the imagery offset
    (cizim.goruntu_kaymasi_m) so they sit on the Google Earth imagery like the user's drawing does.
"""
import os
import numpy as np
from xml.sax.saxutils import escape
from .ortak import Harita, Izdusum, yol, json_oku, yedekle

STIL = ('<Style id="yeni_duz"><LineStyle><color>ff00ffff</color><width>3</width></LineStyle></Style>\n'
        '<Style id="yeni_kesik"><LineStyle><color>ff00a5ff</color><width>2</width></LineStyle></Style>\n'
        '<Style id="orta"><LineStyle><color>ffff00ff</color><width>2</width></LineStyle></Style>\n'
        '<Style id="refuj"><LineStyle><color>ff00c800</color><width>2</width></LineStyle><PolyStyle><color>4000c800</color></PolyStyle></Style>\n'
        '<Style id="yuz_asfalt"><LineStyle><color>ff808080</color><width>1</width></LineStyle><PolyStyle><color>60808080</color></PolyStyle></Style>\n'
        '<Style id="yuz_ada"><LineStyle><color>ff3c78b4</color><width>2</width></LineStyle><PolyStyle><color>803c78b4</color></PolyStyle></Style>\n'
        '<Style id="yuz_bos"><LineStyle><color>ff0000ff</color><width>3</width></LineStyle><PolyStyle><color>c00000ff</color></PolyStyle></Style>\n'
        '<Style id="lamba"><IconStyle><scale>0.6</scale><Icon><href>http://maps.google.com/mapfiles/kml/shapes/placemark_circle.png</href></Icon></IconStyle></Style>\n')
BOLUM = '<Folder><name>Şerit çizgileri (sim)</name>'


def _ozel(A, H, I, kay):
    """Gate lanes + median + shelter folders. kay = (dx, dy) added before converting to lat/lon."""
    co = lambda pts: ' '.join('%.8f,%.8f,0' % (lo, la) for la, lo in zip(*I.xy2ll(np.asarray(pts)[:, 0] + kay[0], np.asarray(pts)[:, 1] + kay[1])))
    notu = A['nizamiye']['not']; yeni = []
    for lid, (m, t) in H.R.items():
        if t.get('type') == 'lanelet' and notu in t.get('note', ''):
            yeni.append((lid, t['note'].split(': ', 1)[1], m))
    yollar = sorted({w for _, _, m in yeni for w in m.values()})
    o = ['<Folder><name>Nizamiye şeritleri</name><open>1</open>\n'
         '<description>Sarı: düz şerit çizgisi, turuncu: kesikli, mor: şerit ortası (yön: çizginin başından sonuna).</description>\n']
    o += ['<Placemark><styleUrl>#%s</styleUrl><LineString><coordinates>%s</coordinates></LineString></Placemark>\n'
          % ('yeni_kesik' if H.WT[w].get('subtype') == 'dashed' else 'yeni_duz', co(H.P(w))) for w in yollar]

    def res(p, n):
        s = np.r_[0, np.cumsum(np.hypot(*np.diff(p[:, :2], axis=0).T))]; s /= s[-1]; u = np.linspace(0, 1, n)
        return np.c_[np.interp(u, s, p[:, 0]), np.interp(u, s, p[:, 1])]
    for lid, ad, m in yeni:
        L, R = H.P(m['left']), H.P(m['right']); n = max(len(L), len(R))
        o.append('<Placemark><name>%s</name><description>lanelet %s</description><styleUrl>#orta</styleUrl><LineString><coordinates>%s</coordinates></LineString></Placemark>\n'
                 % (escape(ad), lid, co((res(L, n) + res(R, n)) / 2)))
    ara = yol(A['yollar']['ara'])
    dp = os.path.join(ara, 'durak.json')
    if os.path.exists(dp):
        d = json_oku(dp)
        o.append('<Placemark><name>otobüs durağı kulübesi (sim)</name><Point><coordinates>%s</coordinates></Point></Placemark>\n' % co([[d['x'], d['y']]]))
    o.append('</Folder>\n')
    yp = os.path.join(ara, 'nizamiye_yuzey.json')
    if os.path.exists(yp):     # what the simulation has: asphalt (grey), islands (brown), bare ground between roads (red)
        y_ = json_oku(yp)
        o.append('<Folder><name>Simülasyon yüzeyleri (nizamiye)</name>\n')
        for ad, st, liste in [('asfalt', 'yuz_asfalt', y_['asfalt']), ('ada (kaldırım + çim / beton)', 'yuz_ada', y_['adalar']), ('AÇIKTA ZEMİN', 'yuz_bos', y_['bos'])]:
            o += ['<Placemark><name>%s</name><styleUrl>#%s</styleUrl><Polygon><outerBoundaryIs><LinearRing><coordinates>%s</coordinates>'
                  '</LinearRing></outerBoundaryIs></Polygon></Placemark>\n' % (ad, st, co(np.array(p))) for p in liste]
        o.append('</Folder>\n')
    rp = os.path.join(ara, 'refuj_ozet.json')
    if os.path.exists(rp):
        r = json_oku(rp); s = np.array(r['sinir_xy']).reshape(-1, 2); s = np.r_[s, s[:1]]
        o.append('<Folder><name>Refüj (orta ada)</name>\n<Placemark><name>refüj kenarı (bordür)</name><styleUrl>#refuj</styleUrl>'
                 '<Polygon><outerBoundaryIs><LinearRing><coordinates>%s</coordinates></LinearRing></outerBoundaryIs></Polygon></Placemark>\n' % co(s))
        o += ['<Placemark><name>%s</name><styleUrl>#lamba</styleUrl><Point><coordinates>%s</coordinates></Point></Placemark>\n'
              % (l['ad'], co([[l['x'], l['y']]])) for l in r['lambalar']]
        o.append('<Folder><name>ağaçlar (%d)</name><visibility>0</visibility>\n' % len(r['agaclar']))
        o += ['<Placemark><name>%s %.0f m</name><visibility>0</visibility><Point><coordinates>%s</coordinates></Point></Placemark>\n'
              % ('ağaç' if t['tur'] == 'agac' else 'çalı', t['boy'], co([[t['x'], t['y']]])) for t in r['agaclar']]
        o.append('</Folder>\n</Folder>\n')
    return o, yeni


def calistir(A):
    H = Harita(yol(A['yollar']['harita'])); I = Izdusum(H)
    hedef = yol(A['yollar']['sim_kml']); eski = open(hedef).read()
    if BOLUM not in eski:
        raise SystemExit('HATA: %s içinde "%s" bölümü yok; bina kısmı korunamadı' % (hedef, BOLUM))
    bas = eski[:eski.index(BOLUM)]
    if 'id="yeni_duz"' not in bas:
        bas = bas.replace('<Style id="yol">', STIL + '<Style id="yol">', 1)
    if 'id="yuz_asfalt"' not in bas:
        bas = bas.replace('<Style id="yol">', ''.join(l + '\n' for l in STIL.split('\n') if 'yuz_' in l) + '<Style id="yol">', 1)
    if 'id="refuj"' not in bas:
        bas = bas.replace('<Style id="yol">', STIL.split('\n', 3)[3] + '<Style id="yol">', 1)
    ozel, yeni = _ozel(A, H, I, (0.0, 0.0))
    yeni_yollar = {w for _, _, m in yeni for w in m.values()}
    eski_yollar = set()
    for lid, (m, t) in H.R.items():
        if t.get('type') == 'lanelet' and t.get('subtype', 'road') == 'road' and 'left' in m:
            eski_yollar |= {m['left'], m['right']}
    eski_yollar -= yeni_yollar
    co = lambda pts: ' '.join('%.8f,%.8f,0' % (lo, la) for la, lo in zip(*I.xy2ll(pts[:, 0], pts[:, 1])))
    o = [bas, BOLUM + '\n']
    o += ['<Placemark><styleUrl>#yol</styleUrl><LineString><coordinates>%s</coordinates></LineString></Placemark>\n' % co(H.P(w)) for w in sorted(eski_yollar)]
    o.append('</Folder>\n'); o += ozel; o.append('</Document></kml>\n')
    yedekle(hedef, A)
    with open(hedef, 'w') as f:
        f.write(''.join(o))
    dx, dy = A['cizim']['goruntu_kaymasi_m']
    ozel_ge, _ = _ozel(A, H, I, (-dx, -dy))
    ge = os.path.join(yol(A['yollar']['ara']), 'google_earth_hizali.kml')
    with open(ge, 'w') as f:
        f.write('<?xml version="1.0" encoding="UTF-8"?>\n<kml xmlns="http://www.opengis.net/kml/2.2"><Document>'
                '<name>Nizamiye + refüj (Google Earth görüntüsüne hizalı)</name>\n'
                '<description>Harita çerçevesinden %.2f m batı, %.2f m güney kaydırıldı: Google Earth görüntüsü ve çizimle aynı hizada.</description>\n'
                % (dx, dy) + STIL + ''.join(ozel_ge) + '</Document></kml>\n')
    import xml.etree.ElementTree as ET
    ET.parse(hedef); ET.parse(ge)
    print(f'kml: {len(eski_yollar)} eski çizgi, {len(yeni)} nizamiye şeridi -> {hedef}\n     Google Earth hizalı kontrol -> {ge}')
