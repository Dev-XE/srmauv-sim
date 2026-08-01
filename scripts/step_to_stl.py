"""Convert a STEP assembly to a single STL mesh, headless via FreeCAD.

Run with FreeCAD's interpreter:
    freecadcmd scripts/step_to_stl.py
(or: freecad -c scripts/step_to_stl.py  on builds without freecadcmd)

Reads:  /home/nayanika/Downloads/AUV Vehicle full assembly v4.step
Writes: src/srmauv_description/meshes/srmauv.stl
"""
import os
import sys

import FreeCAD as App  # noqa: E402
import Part            # noqa: E402
import Mesh            # noqa: E402
import MeshPart        # noqa: E402

STEP_IN = "/home/nayanika/Downloads/AUV Vehicle full assembly v4.step"
STL_OUT = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "src", "srmauv_description", "meshes", "srmauv.stl",
)

# Tessellation quality: smaller deflection = finer mesh + bigger file.
# 0.1 mm linear deflection is a good balance for a ~m-scale vehicle.
LINEAR_DEFLECTION = 0.1
ANGULAR_DEFLECTION = 0.5  # radians


def main():
    if not os.path.exists(STEP_IN):
        sys.exit(f"STEP file not found: {STEP_IN}")

    print(f"Loading {STEP_IN} ...")
    doc = App.newDocument("conv")
    Part.insert(STEP_IN, "conv")

    # Collect every solid/shape in the assembly and mesh each one.
    meshes = []
    for obj in doc.Objects:
        shape = getattr(obj, "Shape", None)
        if shape is None or shape.isNull():
            continue
        try:
            m = MeshPart.meshFromShape(
                Shape=shape,
                LinearDeflection=LINEAR_DEFLECTION,
                AngularDeflection=ANGULAR_DEFLECTION,
                Relative=False,
            )
            meshes.append(m)
            print(f"  meshed: {obj.Name} ({m.CountFacets} facets)")
        except Exception as e:  # noqa: BLE001
            print(f"  skip {obj.Name}: {e}")

    if not meshes:
        sys.exit("No meshable geometry found in the STEP file.")

    combined = Mesh.Mesh()
    for m in meshes:
        combined.addMesh(m)

    os.makedirs(os.path.dirname(STL_OUT), exist_ok=True)
    combined.write(STL_OUT)
    print(f"\nWrote {STL_OUT}")
    print(f"Total facets: {combined.CountFacets}")
    size_mb = os.path.getsize(STL_OUT) / 1e6
    print(f"File size: {size_mb:.1f} MB")
    if size_mb > 50:
        print("WARNING: large mesh. Increase LINEAR_DEFLECTION to simplify.")


main()
