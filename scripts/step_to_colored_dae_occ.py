"""STEP -> colored COLLADA (.dae), headless via OpenCASCADE (OCP).

Walks the XCAF assembly tree, reads per-prototype/per-face colors (which the
located compound loses), meshes each prototype once, applies instance
transforms, groups triangles by color, and writes a multi-material .dae.

    python3 scripts/step_to_colored_dae_occ.py
"""
import os
import numpy as np

from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TDocStd import TDocStd_Document
from OCP.XCAFApp import XCAFApp_Application
from OCP.XCAFDoc import XCAFDoc_DocumentTool, XCAFDoc_ColorType
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDF import TDF_LabelSequence, TDF_Label
from OCP.Quantity import Quantity_Color
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED
from OCP.TopoDS import TopoDS
from OCP.BRep import BRep_Tool
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.TopLoc import TopLoc_Location

import collada

STEP_IN = "/home/nayanika/Downloads/AUV Vehicle full assembly v4.step"
DAE_OUT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                       "src", "srmauv_description", "meshes", "srmauv_colored.dae")
LIN_DEFLECT = 0.5    # mm; coarser than the STL run -> smaller file
ANG_DEFLECT = 0.5    # rad
DEFAULT_COLOR = (0.627, 0.627, 0.627)  # steel-satin grey for uncolored faces

CTYPES = [XCAFDoc_ColorType.XCAFDoc_ColorSurf,
          XCAFDoc_ColorType.XCAFDoc_ColorGen]


# Module-level refs so the XCAF document is not garbage-collected (which would
# silently invalidate the shape/color tools -> GetFreeShapes returns 0).
_KEEP = {}


def load():
    reader = STEPCAFControl_Reader()
    reader.SetColorMode(True)
    reader.SetNameMode(True)
    reader.ReadFile(STEP_IN)
    app = XCAFApp_Application.GetApplication_s()
    doc = TDocStd_Document(TCollection_ExtendedString("MDTV-XCAF"))
    app.InitDocument(doc)
    reader.Transfer(doc)
    _KEEP["reader"] = reader
    _KEEP["app"] = app
    _KEEP["doc"] = doc
    main = doc.Main()
    return (XCAFDoc_DocumentTool.ShapeTool_s(main),
            XCAFDoc_DocumentTool.ColorTool_s(main))


ST = CT = None


def face_color(face, fallback):
    col = Quantity_Color()
    for ct in CTYPES:
        if CT.GetColor(face, ct, col):
            return (round(col.Red(), 4), round(col.Green(), 4), round(col.Blue(), 4))
    return fallback


def shape_color(shp):
    col = Quantity_Color()
    for ct in CTYPES:
        if CT.GetColor(shp, ct, col):
            return (round(col.Red(), 4), round(col.Green(), 4), round(col.Blue(), 4))
    return None


def tri_indices(t):
    try:
        a, b, c = t.Get()
        return a, b, c
    except Exception:
        return t.Value(1), t.Value(2), t.Value(3)


# Accumulators: color -> [verts(list of xyz), tris(list of (i,j,k))]
buckets = {}


def add_face(face, trsf, base_color):
    loc = TopLoc_Location()
    tri = BRep_Tool.Triangulation_s(face, loc)
    if tri is None:
        return 0
    ftrsf = loc.Transformation()
    nb = tri.NbNodes()
    pts = []
    for i in range(1, nb + 1):
        p = tri.Node(i)
        p = p.Transformed(ftrsf)   # prototype coords
        p = p.Transformed(trsf)    # world coords (instance placement)
        pts.append((p.X(), p.Y(), p.Z()))
    reversed_ = (face.Orientation() == TopAbs_REVERSED)
    col = face_color(face, base_color)
    if col not in buckets:
        buckets[col] = [[], []]
    vbuf, tbuf = buckets[col]
    off = len(vbuf)
    vbuf.extend(pts)
    n = 0
    for k in range(1, tri.NbTriangles() + 1):
        a, b, c = tri_indices(tri.Triangle(k))
        a, b, c = a - 1 + off, b - 1 + off, c - 1 + off
        if reversed_:
            b, c = c, b
        tbuf.append((a, b, c))
        n += 1
    return n


def walk(label, trsf, depth=0):
    if ST.IsReference_s(label):
        ref = TDF_Label()
        ST.GetReferredShape_s(label, ref)
        # instance placement
        iloc = ST.GetLocation_s(label)
        ntrsf = trsf.Multiplied(iloc.Transformation())
        walk(ref, ntrsf, depth + 1)
        return
    if ST.IsAssembly_s(label):
        comps = TDF_LabelSequence()
        ST.GetComponents_s(label, comps)
        for i in range(1, comps.Length() + 1):
            walk(comps.Value(i), trsf, depth + 1)
        return
    # leaf prototype shape
    shp = ST.GetShape_s(label)
    base = shape_color(shp) or DEFAULT_COLOR
    BRepMesh_IncrementalMesh(shp, LIN_DEFLECT, False, ANG_DEFLECT, True)
    exp = TopExp_Explorer(shp, TopAbs_FACE)
    while exp.More():
        add_face(TopoDS.Face_s(exp.Current()), trsf, base)
        exp.Next()


def main():
    global ST, CT
    ST, CT = load()
    from OCP.gp import gp_Trsf
    roots = TDF_LabelSequence()
    ST.GetFreeShapes(roots)
    print("free shapes:", roots.Length())
    for i in range(1, roots.Length() + 1):
        walk(roots.Value(i), gp_Trsf(), 0)

    print("color groups:", len(buckets))
    total_tris = 0
    for c, (v, t) in sorted(buckets.items(), key=lambda x: -len(x[1][1])):
        print(f"   rgb={c}  verts={len(v)}  tris={len(t)}")
        total_tris += len(t)
    print("total tris:", total_tris)

    # Build COLLADA.
    mesh = collada.Collada()
    geomnodes = []
    for idx, (c, (verts, tris)) in enumerate(buckets.items()):
        varr = np.array(verts, dtype=np.float32).flatten()
        # per-triangle flat normals
        vnp = np.array(verts, dtype=np.float32)
        tnp = np.array(tris, dtype=np.int32)
        normals = []
        nidx = []
        for ti, (a, b, c2) in enumerate(tnp):
            n = np.cross(vnp[b] - vnp[a], vnp[c2] - vnp[a])
            ln = np.linalg.norm(n)
            n = n / ln if ln > 1e-12 else np.array([0, 0, 1.0])
            normals.extend(n.tolist())
            nidx.extend([ti, ti, ti])
        narr = np.array(normals, dtype=np.float32)

        vsrc = collada.source.FloatSource(f"v{idx}", varr, ("X", "Y", "Z"))
        nsrc = collada.source.FloatSource(f"n{idx}", narr, ("X", "Y", "Z"))
        geom = collada.geometry.Geometry(mesh, f"g{idx}", f"g{idx}", [vsrc, nsrc])
        il = collada.source.InputList()
        il.addInput(0, "VERTEX", f"#v{idx}")
        il.addInput(1, "NORMAL", f"#n{idx}")
        # interleave vertex idx and normal idx
        indices = np.empty(len(tnp) * 6, dtype=np.int32)
        flat_t = tnp.flatten()
        flat_n = np.array(nidx, dtype=np.int32)
        indices[0::2] = flat_t
        indices[1::2] = flat_n
        matref = f"mat{idx}"
        triset = geom.createTriangleSet(indices, il, matref)
        geom.primitives.append(triset)
        mesh.geometries.append(geom)

        effect = collada.material.Effect(
            f"eff{idx}", [], "phong",
            diffuse=(c[0], c[1], c[2], 1.0),
            specular=(0.2, 0.2, 0.2, 1.0))
        matl = collada.material.Material(f"mat{idx}", f"mat{idx}", effect)
        mesh.effects.append(effect)
        mesh.materials.append(matl)

        matnode = collada.scene.MaterialNode(matref, matl, inputs=[])
        geomnodes.append(collada.scene.GeometryNode(geom, [matnode]))

    node = collada.scene.Node("srmauv", children=geomnodes)
    myscene = collada.scene.Scene("scene", [node])
    mesh.scenes.append(myscene)
    mesh.scene = myscene
    os.makedirs(os.path.dirname(DAE_OUT), exist_ok=True)
    mesh.write(DAE_OUT)
    print("WROTE", DAE_OUT, round(os.path.getsize(DAE_OUT) / 1e6, 1), "MB")


main()
