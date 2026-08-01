"""Make the colored .dae read true-to-color under Gazebo's dim default light.

Gazebo only adds material *ambient* in shadowed/ambient regions; our materials
had no ambient, so unlit faces went black. Set ambient = diffuse and add a
small emission floor so every part shows its real color. No re-meshing needed.
"""
import collada

DAE = '/home/yashas/NIOT_srmauv-main/src/srmauv_description/meshes/srmauv_colored.dae'

m = collada.Collada(DAE)
for eff in m.effects:
    d = eff.diffuse
    if not d:
        continue
    r, g, b = d[0], d[1], d[2]
    eff.ambient = (r, g, b, 1.0)            # show true color in ambient light
    eff.emission = (0.18 * r, 0.18 * g, 0.18 * b, 1.0)  # visibility floor
    eff.specular = (0.15, 0.15, 0.15, 1.0)
    eff.shininess = 16.0
m.write(DAE)
print("updated", len(m.effects), "materials in", DAE)
