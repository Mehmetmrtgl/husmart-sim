"""ambientCG (CC0) PBR zip'lerinden Beytepe AWSIM sahnesi için HDRP dokuları üretir.
Her set için:  <Ad>_Base.png (renk; yapraklarda alfa=opaklık), <Ad>_Normal.png (OpenGL normal),
               <Ad>_Mask.png (HDRP mask map: R metal=0, G AO, B detay=0, A pürüzsüzlük=1-roughness)
Kullanım: python3 -I make_beytepe_textures.py <zip_klasörü> <çıktı_klasörü>
Kaynak: https://ambientcg.com (CC0 1.0)
"""
import sys, zipfile, io, os
from PIL import Image, ImageOps

SETS = {  # çıktı adı: ambientCG kimliği
    "Asphalt": "Asphalt033",
    "ConcreteRoad": "Concrete046",
    "ConcreteLight": "Concrete049",
    "Grass": "Grass004",
    "Bark": "Bark016",
    "Leaves": "LeafSet010",
}

src, out = sys.argv[1], sys.argv[2]
os.makedirs(out, exist_ok=True)
for name, aid in SETS.items():
    z = zipfile.ZipFile(os.path.join(src, f"{aid}_2K-JPG.zip"))
    def load(sfx, mode):
        fn = f"{aid}_2K-JPG_{sfx}.jpg"
        if fn not in z.namelist():
            return None
        return Image.open(io.BytesIO(z.read(fn))).convert(mode)
    color = load("Color", "RGB")
    opac = load("Opacity", "L")
    base = color.copy()
    if opac is not None:
        base.putalpha(opac.resize(color.size))
    base.save(os.path.join(out, f"{name}_Base.png"), optimize=True)
    load("NormalGL", "RGB").save(os.path.join(out, f"{name}_Normal.png"), optimize=True)
    rough = load("Roughness", "L").resize(color.size)
    ao = load("AmbientOcclusion", "L")
    ao = ao.resize(color.size) if ao is not None else Image.new("L", color.size, 255)
    zero = Image.new("L", color.size, 0)
    Image.merge("RGBA", (zero, ao, zero, ImageOps.invert(rough))).save(os.path.join(out, f"{name}_Mask.png"), optimize=True)
    print(f"{name:14s} <- {aid}  {color.size}  alfa={'var' if opac is not None else 'yok'}  AO={'var' if ao.getextrema() != (255, 255) else 'yok'}")
with open(os.path.join(out, "LICENSE_ambientCG.txt"), "w") as f:
    f.write("Dokular ambientCG.com kaynaklıdır, CC0 1.0 Universal (kamu malı) lisanslıdır.\n"
            + "\n".join(f"{k}: {v} (https://ambientcg.com/view?id={v})" for k, v in SETS.items()) + "\n")
