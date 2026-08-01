"""Generate the NIOT SAVe mission arena as a Gazebo Classic .world (SDF 1.6).

Exact rulebook dimensions. Top-view plan from the competition drawings.
Frame: X = along the 25 m course (start at -X, octagons at +X);
Y = across the 20 m width; Z up; pool floor z=0, water surface z=2.5 m.

Inter-target distances are placed EXACTLY per the "Targets Distance" sheet:
  first target 5 m from wall; flowers green<->yellow 4.5 m, red<->each 3 m;
  gate 4 m from flowers; bins & cupid 6 m from gate; octagons 5 m beyond.
Component sizes follow the per-target dimension sheets and component list.
"""
import math
import os

OUT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                   "src", "srmauv_description", "worlds", "niot_mission.world")

POOL_L = 25.0
POOL_W = 20.0
DEPTH = 2.5
PIPE2 = 0.0302      # 2" PVC pipe outer radius (~60 mm OD)
PIPE_HALF = 0.0105  # 1/2" support pipe radius
BASE_R = 0.10       # MS base Ø200 -> r=0.1
FLOAT_R = 0.1143    # 9" float -> r=0.1143

# ----- SDF helpers -------------------------------------------------------

def matx(r, g, b, a=1.0, emis=0.12):
    return (f"<material><ambient>{r} {g} {b} {a}</ambient>"
            f"<diffuse>{r} {g} {b} {a}</diffuse>"
            f"<specular>0.1 0.1 0.1 1</specular>"
            f"<emissive>{r*emis} {g*emis} {b*emis} 1</emissive></material>")


def box(sx, sy, sz):
    return f"<box><size>{sx} {sy} {sz}</size></box>"


def cyl(radius, length):
    return f"<cylinder><radius>{radius}</radius><length>{length}</length></cylinder>"


def sphere(radius):
    return f"<sphere><radius>{radius}</radius></sphere>"


def mesh_geom(uri, s=0.001):
    return f"<mesh><uri>{uri}</uri><scale>{s} {s} {s}</scale></mesh>"


# Targets are SOLID (physical contact is part of the mission). Thin support
# posts/tethers stay collision-free (with_collision=False at their call sites)
# so the AUV, cruising above the 0.5 m path plates, never snags on them.
def solid(tag, geom, pose, color, transparency=0.0, with_collision=True):
    out = (f"<visual name='{tag}_v'><pose>{pose}</pose>"
           f"<transparency>{transparency}</transparency>"
           f"<geometry>{geom}</geometry>{matx(*color)}</visual>")
    if with_collision:
        out += (f"<collision name='{tag}_c'><pose>{pose}</pose>"
                f"<geometry>{geom}</geometry></collision>")
    return out


def model(name, x, y, z, parts, yaw=0.0, static=True):
    return (f"<model name='{name}'>"
            f"<static>{'true' if static else 'false'}</static>"
            f"<pose>{x} {y} {z} 0 0 {yaw}</pose>"
            f"<link name='link'>{''.join(parts)}</link></model>\n")


# ----- colors ------------------------------------------------------------
ORANGE = (0.95, 0.45, 0.05)
RED = (0.85, 0.1, 0.1)
GREEN = (0.1, 0.7, 0.15)
YELLOW = (0.95, 0.85, 0.1)
BLUE = (0.1, 0.35, 0.8)
WHITE = (0.92, 0.92, 0.92)
GREY = (0.55, 0.55, 0.55)
PURPLE = (0.35, 0.2, 0.55)

models = []

# ---- Pool: floor + 4 walls ----------------------------------------------
t = 0.2
parts = [
    solid("floor", box(POOL_L, POOL_W, 0.1), "0 0 -0.05 0 0 0", (0.8, 0.78, 0.7), with_collision=True),
    solid("wn", box(POOL_L + 2*t, t, DEPTH), f"0 {POOL_W/2+t/2} {DEPTH/2} 0 0 0", (0.85, 0.88, 0.9), with_collision=True),
    solid("ws", box(POOL_L + 2*t, t, DEPTH), f"0 {-POOL_W/2-t/2} {DEPTH/2} 0 0 0", (0.85, 0.88, 0.9), with_collision=True),
    solid("we", box(t, POOL_W, DEPTH), f"{POOL_L/2+t/2} 0 {DEPTH/2} 0 0 0", (0.85, 0.88, 0.9), with_collision=True),
    solid("ww", box(t, POOL_W, DEPTH), f"{-POOL_L/2-t/2} 0 {DEPTH/2} 0 0 0", (0.85, 0.88, 0.9), with_collision=True),
]
models.append(model("niot_pool", 0, 0, 0, parts))

# ---- Water volume (translucent, visual only) ----------------------------
models.append(model("water", 0, 0, 0, [
    (f"<visual name='water'><pose>0 0 {DEPTH/2} 0 0 0</pose><transparency>0.8</transparency>"
     f"<geometry>{box(POOL_L, POOL_W, DEPTH)}</geometry>{matx(0.1,0.4,0.6,1.0)}</visual>")]))

# ---- positions (exact distances) ----------------------------------------
# West wall inner face at X = -12.5; first target 5 m in -> X = -7.5
RED_X = -7.5
DX_RG = math.sqrt(3.0**2 - 2.25**2)        # red->green dX for 3.0 m (=1.984)
GY_X = RED_X + DX_RG                        # green/yellow line
GATE_X = GY_X + 4.0                         # 4 m from flowers
DX_GATE_BIN = math.sqrt(6.0**2 - 3.0**2)    # 6 m at Y=+/-3 -> dX (=5.196)
BIN_X = GATE_X + DX_GATE_BIN
# octagons 5 m further along the gate->bin diagonal direction (0.866, 0.5)
ux, uy = DX_GATE_BIN/6.0, 3.0/6.0
OCT_X = BIN_X + 5.0*ux
OCT_Y = 3.0 + 5.0*uy

# ---- Flowers: SOLID buoys on true-length poles (2.1 / 2.5 / 3.0 m) -------
# Static + solid: the AUV registers a real touch by grazing right beside a
# buoy (the autopilot routes past, not head-on) — robust, no unstable joints.
def flower(name, x, y, color, pole):
    return model(name, x, y, 0, [
        solid("base", cyl(BASE_R, 0.01), "0 0 0.005 0 0 0", GREY),
        solid("post", cyl(PIPE_HALF, pole), f"0 0 {pole/2} 0 0 0", GREY),
        solid("ball", sphere(FLOAT_R), f"0 0 {pole+FLOAT_R} 0 0 0", color),
    ])

models.append(flower("flower_red", RED_X, 0.0, RED, 2.1))
models.append(flower("flower_green", GY_X, 2.25, GREEN, 2.5))
models.append(flower("flower_yellow", GY_X, -2.25, YELLOW, 3.0))

# ---- Orange path plates: PVC 1200x150x10 mm on 500 mm support pipes -----
def path_plate(name, x, y, yaw):
    z = 0.5
    parts = [solid("plate", box(1.2, 0.15, 0.01), f"0 0 {z} 0 0 0", ORANGE)]
    for sx in (-0.5, 0.5):
        parts.append(solid(f"post{sx}", cyl(PIPE_HALF, z), f"{sx} 0 {z/2} 0 0 0", GREY, with_collision=False))
        parts.append(solid(f"base{sx}", cyl(0.06, 0.01), f"{sx} 0 0.005 0 0 0", GREY, with_collision=False))
    return model(name, x, y, 0, parts, yaw=yaw)

ang_b = math.atan2(3.0, DX_GATE_BIN)        # branch yaw toward bins/cupid
path_layout = [
    (-8.2, 0.0, 0.0), (-6.6, 0.0, 0.0), (-3.5, 0.0, 0.0), (GATE_X, 0.0, 0.0),
    (GATE_X+1.5, 0.9, ang_b), (GATE_X+3.0, 1.8, ang_b),
    (GATE_X+1.5, -0.9, -ang_b), (GATE_X+3.0, -1.8, -ang_b),
]
for i, (px, py, pa) in enumerate(path_layout):
    models.append(path_plate(f"path_{i}", px, py, pa))

# ---- L'ove Lane: 2" PVC L-bar, 1800 + 1200 mm, 90 deg elbow -------------
# Mission: AUV passes OVER this -> low, roughly horizontal bent bar held on
# ~500 mm support posts. Y-segment crosses the path; X-segment runs along it.
z = 0.5
arm_x = 1.8     # segment along the course
arm_y = 1.2     # segment across the path
parts = [
    solid("base1", cyl(BASE_R, 0.01), "0 0 0.005 0 0 0", GREY),
    solid("post1", cyl(PIPE_HALF, z), f"0 0 {z/2} 0 0 0", GREY, with_collision=False),
    solid("base2", cyl(BASE_R, 0.01), f"1.4 {arm_y/2} 0.005 0 0 0", GREY),
    solid("post2", cyl(PIPE_HALF, z), f"1.4 {arm_y/2} {z/2} 0 0 0", GREY, with_collision=False),
    solid("segY", cyl(PIPE2, arm_y), f"0 0 {z} 1.5708 0 0", WHITE),                 # across path (1.2 m)
    solid("elbow", sphere(PIPE2*1.3), f"0 {arm_y/2} {z} 0 0 0", WHITE),
    solid("segX", cyl(PIPE2, arm_x), f"{arm_x/2} {arm_y/2} {z} 0 1.5708 0", WHITE),  # along course (1.8 m)
    solid("cap1", sphere(PIPE2*1.1), f"0 {-arm_y/2} {z} 0 0 0", WHITE, with_collision=False),
    solid("cap2", sphere(PIPE2*1.1), f"{arm_x} {arm_y/2} {z} 0 0 0", WHITE, with_collision=False),
]
models.append(model("love_lane", GATE_X, 0.0, 0, parts))

# ---- Path & Bins: 2 bins (O, X), plate 360x420, on 4 x 500 mm posts -----
def bin_model(name, x, y, mark):
    bw, bd, wh, wt = 0.36, 0.42, 0.12, 0.02
    z = 0.5
    parts = [solid("bottom", box(bw, bd, wt), f"0 0 {z} 0 0 0", PURPLE)]
    for j, (dx, dy, wx, wy) in enumerate([
            (bw/2, 0, wt, bd), (-bw/2, 0, wt, bd),
            (0, bd/2, bw, wt), (0, -bd/2, bw, wt)]):
        parts.append(solid(f"w{j}", box(wx, wy, wh), f"{dx} {dy} {z+wh/2} 0 0 0", PURPLE))
    for cx in (-bw/2+0.02, bw/2-0.02):
        for cy in (-bd/2+0.02, bd/2-0.02):
            parts.append(solid(f"pt{cx}{cy}", cyl(PIPE_HALF, z), f"{cx} {cy} {z/2} 0 0 0", GREY, with_collision=False))
    if mark == "X":
        parts.append(solid("m1", box(0.30, 0.03, 0.005), f"0 0 {z+0.01} 0 0 0.785", YELLOW, with_collision=False))
        parts.append(solid("m2", box(0.30, 0.03, 0.005), f"0 0 {z+0.01} 0 0 -0.785", YELLOW, with_collision=False))
    else:  # O = yellow square frame
        for dx, dy, wx, wy in [(0.13, 0, 0.03, 0.26), (-0.13, 0, 0.03, 0.26),
                               (0, 0.13, 0.26, 0.03), (0, -0.13, 0.26, 0.03)]:
            parts.append(solid(f"o{dx}{dy}", box(wx, wy, 0.005), f"{dx} {dy} {z+0.01} 0 0 0", YELLOW, with_collision=False))
    return model(name, x, y, 0, parts)

models.append(bin_model("bin_O", BIN_X, 3.3, "O"))
models.append(bin_model("bin_X", BIN_X, 2.7, "X"))

# ---- Cupid: 610x610 plates with real heart cutouts (green over red) -----
# Mesh: 610x610x10 mm plate with parametric heart hole (mesh collision keeps
# the hole open so a torpedo can pass through). On a 1000 mm MS support pipe.
CUPID_MESH = "model://srmauv_description/meshes/cupid_heart_plate.stl"
mg = mesh_geom(CUPID_MESH)
parts = [
    solid("base", cyl(BASE_R, 0.01), "0 -1.0 0.005 0 0 0", GREY),
    solid("hpipe", cyl(PIPE_HALF, 1.0), "0 -0.5 0.3 1.5708 0 0", GREY, with_collision=False),
    solid("post", cyl(PIPE_HALF, 0.5), "0 0 0.25 0 0 0", GREY, with_collision=False),
    solid("red", mg, "0 0 0.805 0 0 0", RED),      # lower square (z 0.50-1.11)
    solid("green", mg, "0 0 1.415 0 0 0", GREEN),  # upper square (z 1.11-1.72)
]
models.append(model("cupid", BIN_X, -3.0, 0, parts))

# ---- Candy & delivery octagons: 2.7 m across, floats, at surface --------
def octagon(name, x, y):
    across = 2.7
    R = (across / 2) / math.cos(math.radians(22.5))
    edge = 2 * R * math.sin(math.radians(22.5))
    z = DEPTH                                          # floats at surface
    parts = [
        solid("tether", cyl(0.02, z), f"0 0 {z/2} 0 0 0", GREY, with_collision=False),
        solid("candy", cyl(BASE_R, 0.02), f"0 0 {z} 0 0 0", (0.2, 0.7, 0.7)),
    ]
    for k in range(8):
        a0 = math.radians(22.5 + 45 * k)
        a1 = math.radians(22.5 + 45 * (k + 1))
        vx0, vy0 = R*math.cos(a0), R*math.sin(a0)
        vx1, vy1 = R*math.cos(a1), R*math.sin(a1)
        mx, my = (vx0+vx1)/2, (vy0+vy1)/2
        ang = math.atan2(vy1-vy0, vx1-vx0)
        parts.append(solid(f"seg{k}", cyl(PIPE2, edge), f"{mx} {my} {z} 0 1.5708 {ang}", BLUE))
    for k in range(0, 8, 2):
        a = math.radians(22.5 + 45 * k)
        parts.append(solid(f"fl{k}", sphere(0.12), f"{R*math.cos(a)} {R*math.sin(a)} {z} 0 0 0", YELLOW))
    return model(name, x, y, 0, parts)

models.append(octagon("octagon_left", OCT_X, OCT_Y))
models.append(octagon("octagon_right", OCT_X, -OCT_Y))

# ---- assemble -----------------------------------------------------------
world = f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="niot_mission">
    <scene>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.7 0.85 0.95 1</background>
      <shadows>true</shadows>
    </scene>
    <include><uri>model://sun</uri></include>
    <light name='fill' type='directional'>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.5 0.5 0.5 1</diffuse><specular>0.1 0.1 0.1 1</specular>
      <direction>0.4 0.4 -1</direction><cast_shadows>false</cast_shadows>
    </light>
    <physics name='default' type='ode'><gravity>0 0 -9.81</gravity></physics>
{"".join(models)}  </world>
</sdf>
"""
os.makedirs(os.path.dirname(OUT), exist_ok=True)
with open(OUT, "w") as f:
    f.write(world)

# report the locked distances
print("wrote", OUT, len(models), "models")
print(f"  red->green   = {math.dist((RED_X,0),(GY_X,2.25)):.3f} m (spec 3.0)")
print(f"  green<->yellow = {2*2.25:.3f} m (spec 4.5)")
print(f"  flowers->gate = {GATE_X-GY_X:.3f} m (spec 4.0)")
print(f"  gate->bins   = {math.dist((GATE_X,0),(BIN_X,3.0)):.3f} m (spec 6.0)")
print(f"  bins->octagon = {math.dist((BIN_X,3.0),(OCT_X,OCT_Y)):.3f} m (spec 5.0)")
print(f"  first target from wall = {RED_X-(-POOL_L/2):.3f} m (spec 5.0)")
