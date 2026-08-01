"""Generate the Cupid plate mesh: a 610 x 610 x 10 mm PVC plate with a real
heart-shaped cutout (460 mm wide, centered, 75 mm side margins), via OpenCASCADE.

Output: src/srmauv_description/meshes/cupid_heart_plate.stl  (units: mm)
Referenced twice in the world (green top, red bottom). Plate lies in the
Y-Z plane (normal along +X) so it faces an AUV approaching along +X.
"""
import math
import os

from OCP.gp import gp_Pnt, gp_Vec
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakePolygon, BRepBuilderAPI_MakeFace
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
from OCP.BRepPrimAPI import BRepPrimAPI_MakePrism
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.StlAPI import StlAPI_Writer

OUT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                   "src", "srmauv_description", "meshes", "cupid_heart_plate.stl")

SQ = 610.0          # square side (mm)
THK = 10.0          # plate thickness (mm)
HEART_W = 460.0     # heart width (mm) -> 75 mm margins each side

half = SQ / 2.0


def p(y, z):
    return gp_Pnt(0.0, y, z)          # YZ plane, x=0


# ---- square face --------------------------------------------------------
sq = BRepBuilderAPI_MakePolygon()
for y, z in [(-half, -half), (half, -half), (half, half), (-half, half)]:
    sq.Add(p(y, z))
sq.Close()
square_face = BRepBuilderAPI_MakeFace(sq.Wire(), True).Face()

# ---- heart wire (parametric, point downward) ----------------------------
N = 160
raw = []
for i in range(N):
    t = 2 * math.pi * i / N
    wy = 16 * math.sin(t) ** 3
    wz = 13 * math.cos(t) - 5 * math.cos(2*t) - 2 * math.cos(3*t) - math.cos(4*t)
    raw.append((wy, wz))
scale = HEART_W / 32.0                 # param width spans -16..16 (=32)
zc = (max(z for _, z in raw) + min(z for _, z in raw)) / 2.0
heart = BRepBuilderAPI_MakePolygon()
for wy, wz in raw:
    heart.Add(p(scale * wy, scale * (wz - zc)))
heart.Close()
heart_face = BRepBuilderAPI_MakeFace(heart.Wire(), True).Face()

# ---- cut heart from square, extrude to a 10 mm solid --------------------
holed = BRepAlgoAPI_Cut(square_face, heart_face).Shape()
solid = BRepPrimAPI_MakePrism(holed, gp_Vec(THK, 0.0, 0.0)).Shape()

BRepMesh_IncrementalMesh(solid, 1.0, False, 0.3, True)
os.makedirs(os.path.dirname(OUT), exist_ok=True)
w = StlAPI_Writer()
ok = w.Write(solid, OUT)
print("wrote" if ok else "FAILED", OUT, round(os.path.getsize(OUT)/1024, 1), "KB")
