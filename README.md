# MoonMapper — ROS 2 workspace (bachelor)

MoonMapper er en simulert rover-plattform for **autonom utforskning**, **kartlegging (RTAB-Map)** og **Nav2** i Gazebo Sim. **Triad + ML** (fysisk robot) ligger i `ml/`, `arduino/`, `scripts/predict.py` — ROS ML-pakker er arkivert.

Gamle, eksperimentelle og dupliserte filer er flyttet til **`arkiverte_koder/`** (ingenting slettet). Se `CLEANUP_REPORT.md` og `arkiverte_koder/README.md`.

---

## Aktive ROS 2-pakker (`src/`)

| Pakke | Rolle |
|-------|--------|
| `moonmapper_description` | URDF/Xacro, Gazebo-verdener, `gazebo_rover` |
| `moonmapper_bringup` | `sim_rover_clean`, `cmd_vel_odom_relay` |
| ~~`moonmapper_gz_sensors`~~ | Arkivert — syntetisk Triad Gazebo-plugin (`arkiverte_koder/gamle_ros_pakker/`) |
| `moonmapper_autonomy` | `depth_to_scan_node`, `safety_obstacle_node` (aktive; øvrige noder i arkiv) |
| `moonmapper_nav2` | RTAB-Map + Nav2 + `frontier_explorer` |
| ~~`moonmapper_interfaces`~~ | Arkivert — Triad/ML-meldinger (`gamle_ros_pakker/`) |
| ~~`moonmapper_msgs`~~ | Arkivert — `TriadSpectrum` (kun sim-plugin) |
| ~~`moonmapper_localization`~~ | Arkivert — legacy EKF-yaml (`arkiverte_koder/gamle_ros_pakker/`) |
| ~~`moonmapper_ml`~~ | Arkivert — placeholder `ml_inference_node` |
| ~~`moonmapper_perception`~~ | Arkivert — placeholder serial/feature-noder |

---

## Bygg

```bash
cd moonmapper_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## Anbefalt kjøring

### 1. Ren sim (Gazebo + sensorer)

```bash
ros2 launch moonmapper_bringup sim_rover_clean.launch.py use_sim_time:=true
```

### 2. Full autonom utforskning (sim + RTAB-Map + Nav2 + frontier)

```bash
ros2 launch moonmapper_nav2 autonomous_exploration_full.launch.py \
  use_sim_time:=true world_preset:=earth_explore \
  start_sim:=true start_slam:=true start_nav2:=true start_explorer:=true \
  navigation_stack_delay:=12.0 explorer_extra_delay_sec:=5.0
```

Helsesjekk (annet terminalvindu):

```bash
ros2 run moonmapper_nav2 check_autonomous_exploration_stack.sh
```

### 3. ML / Triad (offline)

```bash
pip install -r requirements-ml.txt
python ml/training/extract_triad_features.py --metadata ml/datasets/metadata.csv --raw-dir ml/datasets
python ml/training/train_random_forest.py --input ml/datasets/processed/train.csv --train-all
python scripts/predict.py --raw ml/datasets/live/T0001_triad_raw.csv
```

Se `maskinlaering.md` og `docs/triaddokumentasjon.md`.

---

## Viktige filer og dokumentasjon

| Fil | Innhold |
|-----|---------|
| `digital_tvilling.md` | Teknisk kap. 6 — Gazebo, URDF, sensorer, cmd_vel |
| `maskinlaering.md` | Teknisk kap. 10 — Triad, features, Random Forest |
| `bachelor_rapport_utkast.md` | Rapportstruktur (navigasjon/sim) |
| `bachelor_rapport_maskinlaering_utkast.md` | Rapportstruktur (ML) |
| `CLEANUP_REPORT.md` | Oppryddingslogg |

---

## Mapper utenfor `src/`

| Mappe | Rolle |
|-------|--------|
| `ml/` | Datasett, trening, modeller |
| `scripts/` | `predict.py`, feature-ekstraksjon |
| `docs/` | Triad-dokumentasjon |
| `arduino/` | Datainnsamling Triad |
| `arkiverte_koder/` | Arkivert historikk |

---

## Verdener (`world_preset`)

| Preset | SDF |
|--------|-----|
| `earth_explore` | `earth_arena_explore.sdf` (anbefalt for utforskning) |
| `earth` | `earth_arena.sdf` |
| `moon` | `moon_arena.sdf` |

---

## Validering etter opprydding

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch moonmapper_bringup sim_rover_clean.launch.py use_sim_time:=true
# annet vindu:
ros2 launch moonmapper_nav2 autonomous_exploration_full.launch.py \
  use_sim_time:=true world_preset:=earth_explore \
  start_sim:=true start_slam:=true start_nav2:=true start_explorer:=true
```
