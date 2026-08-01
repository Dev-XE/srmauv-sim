# SAVe AUV — NIOT Student AUV Simulation

Gazebo Classic 11 + ROS 2 Humble simulation of the **SRM University AUV** for the
**NIOT SAVe** (Student Autonomous underwater Vehicle) competition. The repository
contains the robot description, the colored vehicle mesh, and a dimensionally
faithful reconstruction of the **NIOT pool mission arena**.

> **Status:** the `srmauv` model is currently a **static mesh baseline** (correct
> geometry, colors, and a box collision). **Dynamic underwater physics (buoyancy,
> drag, added mass) and thruster integration are the next milestone** — the vehicle
> does not yet float or self-propel.

---

## Repository layout

```
save_auv/
├── src/
│   └── srmauv_description/          # ament_cmake package: description + worlds
│       ├── urdf/
│       │   └── srmauv.urdf.xacro    # AUV: colored mesh visual + box collision/inertia
│       ├── meshes/                  # vehicle + target meshes
│       │   ├── srmauv_colored.dae   # full colored vehicle (from STEP via OpenCASCADE)
│       │   └── cupid_heart_plate.stl# Cupid target plate w/ real heart cutout
│       ├── worlds/
│       │   ├── niot_mission.world   # full NIOT arena (rulebook-exact distances)
│       │   └── srmauv.world         # plain bright world (AUV only)
│       ├── launch/
│       │   ├── niot_mission.launch.py  # arena + AUV  (primary)
│       │   └── gazebo.launch.py        # AUV only
│       ├── CMakeLists.txt
│       └── package.xml
├── scripts/                         # asset-generation tooling (not built)
│   ├── gen_niot_world.py            # generates worlds/niot_mission.world
│   ├── gen_cupid_mesh.py            # generates the heart-cutout plate mesh
│   ├── step_to_stl.py               # STEP → STL (FreeCAD, headless)
│   ├── step_to_colored_dae_occ.py   # STEP → colored DAE (OpenCASCADE/OCP)
│   └── brighten_dae_materials.py    # lifts DAE ambient for Gazebo lighting
├── .gitattributes                   # Git LFS rules for 3D assets
├── .gitignore
└── README.md
```

---

## Prerequisites

| Component | Version |
|-----------|---------|
| OS        | Ubuntu 22.04 (Jammy) |
| ROS 2     | Humble Hawksbill |
| Simulator | Gazebo **Classic** 11 |
| Bridge    | `gazebo_ros_pkgs` (gazebo_ros, gazebo_plugins) |
| VCS       | Git |

Install the ROS/Gazebo dependencies:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-xacro
```

> **Optional — only to *regenerate* assets** (teammates running the sim do **not**
> need these): `pip install --user cadquery-ocp pycollada` and FreeCAD
> (`sudo apt install freecad`). See [Regenerating assets](#regenerating-assets).

---

## Setup

```bash
# 1. Clone the repository
git clone https://github.com/nayanikaprusty518-hue/save_auv.git
cd save_auv

# 2. Build the workspace
source /opt/ros/humble/setup.bash
colcon build

# 3. Source the overlay (do this in every new terminal)
source install/setup.bash
```

> Tip: add the two `source` lines to your `~/.bashrc` so every terminal is ready:
> ```bash
> echo 'source /opt/ros/humble/setup.bash'        >> ~/.bashrc
> echo 'source ~/save_auv/install/setup.bash'      >> ~/.bashrc
> ```

---

## Running the NIOT pool mission map

Launch the full arena with the AUV spawned at the start:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch srmauv_description niot_mission.launch.py
```

Gazebo opens with the **25 × 20 m, 2.5 m-deep pool** and all mission targets at
**rulebook-exact distances**: orange path plates, the red/green/yellow flower
buoys, the L'ove Lane bar, the "O"/"X" bins, the Cupid heart-cutout plates, and
the two surfacing octagons.

Launch just the vehicle (no arena):

```bash
ros2 launch srmauv_description gazebo.launch.py
```

If a previous run left a server holding the port, clear it first:

```bash
pkill -9 -x gzserver; pkill -9 -x gzclient
```

---

## Regenerating assets

All meshes and the world file are reproducible from the scripts. They are only
needed if you change the source CAD or the arena layout:

```bash
python3 scripts/step_to_colored_dae_occ.py   # STEP → colored vehicle DAE
python3 scripts/gen_cupid_mesh.py            # heart-cutout plate STL
python3 scripts/gen_niot_world.py            # regenerate the arena world
colcon build                                 # reinstall updated assets
```

> The source CAD (`AUV Vehicle full assembly v4.step`) is **not** in this repo by
> default. To make regeneration reproducible for the team, commit it under a
> `cad/` directory (it is LFS-tracked via `*.step`).

---

## Notes on large binaries

The colored vehicle mesh (`srmauv_colored.dae`, ~85 MB) and the source CAD
(`cad/*.step`, ~36 MB) are committed directly to Git (both under GitHub's 100 MB
limit). To keep the repository lean as it grows, migrate these to **Git LFS**:

```bash
sudo apt install -y git-lfs && git lfs install
git lfs migrate import --include="*.dae,*.step,*.stl,*.stp,*.obj"
git push --force-with-lease
```

> The unused geometry-only `srmauv.stl` is git-ignored (regenerate with
> `scripts/step_to_stl.py` if needed).

---

## Roadmap

- [x] Vehicle mesh baseline (colored, scaled, box collision)
- [x] NIOT mission arena (rulebook-exact distances, real heart cutouts)
- [ ] **Underwater dynamics** — buoyancy, hydrodynamic drag, added mass
- [ ] **Thrusters** — 6-DOF actuation exposed over ROS 2 topics
- [ ] Sensors — IMU, depth/pressure, cameras
- [ ] Autonomy — path following and mission tasks

---

*Maintainers: SRM AUV team. Built for the NIOT SAVe competition.*
