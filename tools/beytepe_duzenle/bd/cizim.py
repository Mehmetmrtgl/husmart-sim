"""Step "cizim": read the Google Earth KML drawing, convert to the map frame and correct the imagery offset."""
import os
import xml.etree.ElementTree as ET
import numpy as np
from .ortak import Harita, Izdusum, yol, json_yaz, json_oku

NS = '{http://www.opengis.net/kml/2.2}'


def calistir(A):
    H = Harita(yol(A['yollar']['temel_harita'])); I = Izdusum(H)
    dx, dy = A['cizim']['goruntu_kaymasi_m']
    root = ET.parse(yol(A['yollar']['cizim_kml'])).getroot()
    out = {}
    for pm in root.iter(NS + 'Placemark'):
        ad = pm.find(NS + 'name').text.strip()
        for tur in ('LineString', 'Polygon', 'Point'):
            e = pm.find('.//' + NS + tur)
            if e is None:
                continue
            co = e.find('.//' + NS + 'coordinates').text.split()
            ll = np.array([[float(a) for a in s.split(',')[:2]] for s in co])
            xy = I.ll2xy(ll[:, 0], ll[:, 1]) + [dx, dy]
            out[ad] = [tur, xy.tolist()]
    hedef = os.path.join(yol(A['yollar']['ara']), 'cizim_xy.json')
    json_yaz(hedef, out)
    gerekli = list(A['cizim']['rota'].values()) + A['cizim']['asfalt'] + A['cizim']['ada'] + [A['cizim']['durak']]
    eksik = [g for g in gerekli if g not in out]
    print(f'cizim: {len(out)} öğe -> {hedef} (kayma {dx:+.2f} m doğu, {dy:+.2f} m kuzey)')
    if eksik:
        raise SystemExit('HATA: KML\'de bulunamayan öğeler (ayarlar.yaml cizim bölümüyle aynı ad olmalı): ' + ', '.join(eksik))
    return out


def oku(A):
    return json_oku(os.path.join(yol(A['yollar']['ara']), 'cizim_xy.json'))
