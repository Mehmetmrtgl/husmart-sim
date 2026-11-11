#!/usr/bin/env python3
"""Tek parça PCD haritasını Autoware'in parçalı (dinamik yüklenen) harita biçimine böler.
Girdi: binary_compressed (LZF) ya da binary PCD, alanlar x y z (float32).
Çıktı: OUT/pointcloud_map/<ad>_<i>_<j>.pcd (binary) + OUT/pointcloud_map_metadata.yaml
Kullanım: pcd_karola.py girdi.pcd OUT [karo_m=50]"""
import os, sys, struct
import numpy as np

src, out = sys.argv[1], sys.argv[2]
G = float(sys.argv[3]) if len(sys.argv) > 3 else 50.0


def lzf_decompress(data, out_len):
    o = bytearray(out_len); ip = 0; op = 0; n = len(data)
    while ip < n:
        c = data[ip]; ip += 1
        if c < 32:                      # düz bayt dizisi
            c += 1
            o[op:op + c] = data[ip:ip + c]; ip += c; op += c
        else:                           # geri başvuru
            ln = c >> 5
            if ln == 7:
                ln += data[ip]; ip += 1
            ref = op - ((c & 0x1f) << 8) - data[ip] - 1; ip += 1
            ln += 2
            if ref + ln <= op:          # örtüşmüyor: dilimle kopyala
                o[op:op + ln] = o[ref:ref + ln]
            else:                       # örtüşüyor: bayt bayt
                for k in range(ln):
                    o[op + k] = o[ref + k]
            op += ln
    if op != out_len:
        raise ValueError(f"LZF: {op} != {out_len}")
    return bytes(o)


with open(src, "rb") as f:
    raw = f.read()
hdr_end = raw.index(b"\nDATA ") + 1
line_end = raw.index(b"\n", hdr_end)
header = raw[:hdr_end].decode()
data_kind = raw[hdr_end:line_end].decode().split()[1]
h = {}
for line in header.splitlines():
    if line and not line.startswith("#"):
        k, *v = line.split(); h[k] = v
fields = h["FIELDS"]; sizes = list(map(int, h["SIZE"])); types = h["TYPE"]; npts = int(h["POINTS"][0])
assert fields[:3] == ["x", "y", "z"] and all(s == 4 for s in sizes) and all(t == "F" for t in types), (fields, sizes, types)
nf = len(fields); body = raw[line_end + 1:]
if data_kind == "binary_compressed":
    csize, usize = struct.unpack("<II", body[:8])
    dec = lzf_decompress(body[8:8 + csize], usize)
    arr = np.frombuffer(dec, dtype="<f4").reshape(nf, npts).T     # sütun düzeni (SoA)
elif data_kind == "binary":
    arr = np.frombuffer(body[:npts * 4 * nf], dtype="<f4").reshape(npts, nf)
else:
    raise SystemExit("desteklenmeyen PCD: " + data_kind)
xyz = np.ascontiguousarray(arr[:, :3], dtype="<f4")
ok = np.isfinite(xyz).all(1); xyz = xyz[ok]
print(f"{npts} nokta okundu ({data_kind}), sonlu {len(xyz)}; x {xyz[:,0].min():.1f}..{xyz[:,0].max():.1f} "
      f"y {xyz[:,1].min():.1f}..{xyz[:,1].max():.1f} z {xyz[:,2].min():.1f}..{xyz[:,2].max():.1f}")

ix = np.floor(xyz[:, 0] / G).astype(np.int64); iy = np.floor(xyz[:, 1] / G).astype(np.int64)
order = np.lexsort((iy, ix)); xyz, ix, iy = xyz[order], ix[order], iy[order]
key = ix * 1000003 + iy
starts = np.r_[0, np.flatnonzero(np.diff(key)) + 1, len(key)]
pdir = os.path.join(out, "pointcloud_map"); os.makedirs(pdir, exist_ok=True)
meta = [f"x_resolution: {G:.1f}", f"y_resolution: {G:.1f}"]
tot = 0; zr = 0.0
for a, b in zip(starts[:-1], starts[1:]):
    i, j = int(ix[a]), int(iy[a]); P = xyz[a:b]; tot += len(P)
    name = f"pointcloud_map_{i}_{j}.pcd"
    hd = ("# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
          f"WIDTH {len(P)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(P)}\nDATA binary\n")
    with open(os.path.join(pdir, name), "wb") as f:
        f.write(hd.encode()); f.write(P.tobytes())
    meta.append(f"{name}: [{int(i * G)}, {int(j * G)}]")   # Autoware tamsayı bekler (yaml-cpp as<vector<int>>)
    zr = max(zr, float(P[:, 2].max() - P[:, 2].min()))
with open(os.path.join(out, "pointcloud_map_metadata.yaml"), "w") as f:
    f.write("\n".join(meta) + "\n")
print(f"{len(starts) - 1} karo ({G:.0f} m), toplam {tot} nokta, karo içi en büyük z aralığı {zr:.1f} m -> {out}")
