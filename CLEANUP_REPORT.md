# Workspace cleanup report

**Dato:** 2026-05-17  
**Branch:** `nav2/odom-baseline`  
**Metode:** Analyse av avhengigheter → `git mv` til `arkiverte_koder/` → minimale build-fikser → `colcon build` på aktive pakker.

## Formål

Workspacet er ryddet for **bachelorinnlevering** uten å slette filer. Alt som ikke inngår i den aktive leveransen (Gazebo-sim, RTAB-Map, Nav2, frontier explorer, ML-pipeline, rapportdokumentasjon) er flyttet til `arkiverte_koder/`. Git-historikk er bevart der filene var sporet.

## Beholdt aktiv struktur

### ROS 2-pakker (`src/`)

| Pakke | Hvorfor beholdt |
|-------|-----------------|
| `moonmapper_description` | URDF, Gazebo-verdener, plugins, `gazebo_rover.launch.py` |
| `moonmapper_bringup` | `sim_rover_clean`, sensorbroer, cmd_vel-relay |
| ~~`moonmapper_gz_sensors`~~ | Arkivert — syntetisk Triad-plugin |
| `moonmapper_autonomy` | depth→scan, safety (brukes av Nav2-launch) |
| `moonmapper_nav2` | RTAB-Map, Nav2, frontier explorer |
| ~~`moonmapper_interfaces`~~ | Arkivert — Triad/ML-meldinger |
| ~~`moonmapper_msgs`~~ | Arkivert — `TriadSpectrum` (gz_sensors) |
| ~~`moonmapper_localization`~~ | Arkivert → `gamle_ros_pakker/moonmapper_localization/` |
| ~~`moonmapper_ml`~~ | Arkivert — placeholder inferens |
| ~~`moonmapper_perception`~~ | Arkivert — placeholder serial/feature |

### Aktive Nav2-launch (i `src/moonmapper_nav2/launch/`)

- `autonomous_exploration_full.launch.py`
- `rtabmap_sim.launch.py`
- `rtabmap_real.launch.py`
- `nav2_rtabmap_navigation.launch.py`
- `_nav2_rtabmap_common.py`

### Aktive Nav2-config

- `nav2_params_rtabmap_sim.yaml`
- `nav2_params_rtabmap_real.yaml`
- `frontier_explorer.yaml`
- `rtabmap_real_params.yaml`

### Aktive bringup-launch

- `sim_rover_clean.launch.py` (+ `cmd_vel_odom_relay` node)

### Øvrig aktivt

| Sti | Rolle |
|-----|--------|
| `ml/datasets/raw/`, `metadata.csv`, `processed/train.csv` | ML-trening |
| `ml/training/`, `scripts/` | Feature-ekstraksjon og prediksjon |
| `docs/` | Triad-dokumentasjon |
| `arduino/` | Datainnsamling |
| `digital_tvilling.md`, `maskinlaering.md`, `bachelor_rapport*.md` | Bachelor |
| `README.md` | Hovedoppstart |

## Kodeendringer (nødvendig for build etter flytting)

| Fil | Endring |
|-----|---------|
| `src/moonmapper_bringup/setup.py` | Fjernet Webots/real_robot-launch fra install-liste |
| `src/moonmapper_bringup/package.xml` | Fjernet avhengighet til `moonmapper_webots`, `moonmapper_control`, `moonmapper_slam` |
| `src/moonmapper_nav2/CMakeLists.txt` | Fjernet install av arkiverte debug-/sim-scripts |
| `src/moonmapper_autonomy/setup.py` | Installerer launch kun hvis `launch/*.launch.py` finnes |
| `digital_tvilling.md` | Oppdatert Webots-henvisning til arkiv |
| `src/moonmapper_description/CMakeLists.txt` | Fjernet `models/` fra install |
| `src/moonmapper_description/package.xml` | Fjernet `joint_state_publisher_gui` |

## Flyttet til arkiv — oversikt

**Totalt:** ca. 170 git-renames + nye arkivfiler. Full liste: `git log --follow` / `git status` etter commit.

| Kategori | Eksempler | Arkivmappe |
|----------|-----------|------------|
| Gamle dokumenter | `Kodedokumentasjone.md`, `simulasjon_TEST.md` | `gamle_dokumentasjon_utkast/` |
| Hele ROS-pakker | `ros_tcp_endpoint`, `moonmapper_slam`, `moonmapper_control`, `moonmapper_mapping` | `gamle_ros_pakker/` |
| Webots | `sim.launch.py` | `gamle_webots_filer/` |
| Nav2 test/baseline | `nav2_odom_baseline`, `nav2_static_map*`, `nav2_rtabmap_exploration*` | `gamle_nav2_tester/launch/` |
| Nav2 gamle YAML | `nav2_params_moonmapper.yaml`, `frontier_params_*`, odom/amcl-varianter | `gamle_nav2_tester/config/` |
| Nav2 debug | `sim_6wd_motion_test.py`, `nav2_replan_monitor_node.py`, … | `gamle_nav2_tester/nodes/`, `scripts/` |
| slam_toolbox / kart | mapping launches, maps, pad_occupancy_map | `gamle_mapping/`, `gamle_ros_pakker/moonmapper_mapping` |
| Autonomy test-launch | `depth_scan_safety_test`, `reactive_avoidance`, … | `gamle_sensor_tester/` |
| ML backup | `raw_backup_before_equal_rows/` (52 CSV) | `gamle_ml_eksperimenter/` |
| Duplikat/debug | `FARAH.ino`, TF `.gv`/`.pdf`, `.deb` | `duplikater/`, `midlertidige_debug_filer/` |
| Description (URDF/config) | `moonmapper.urdf.xacro`, EKF-yaml, `replan_test_block`, … | `gamle_urdf/`, `gamle_config/`, `gamle_models/`, … under `moonmapper_description/` |
| Sim Triad (Gazebo) | `moonmapper_gz_sensors`, `moonmapper_msgs` | `gamle_ros_pakker/` |
| Fysisk Triad + ROS ML | `moonmapper_interfaces`, `moonmapper_perception`, `moonmapper_ml` | `gamle_ros_pakker/` |

Detaljert fil-for-fil liste:

```bash
git diff --cached --name-status   # etter staging
# eller etter commit:
git show --name-status --oneline HEAD
```

## Usikre filer (ikke flyttet)

| Fil / område | Begrunnelse |
|--------------|-------------|
| ~~`moonmapper_localization`~~ | Hele pakken arkivert 2026-05-17 (legacy `ekf.yaml`; aktiv sim = RTAB-Map + `/odom`) |
| ~~`moonmapper_autonomy` noder uten launch~~ | Flyttet 2026-05-17 til `arkiverte_koder/gamle_nodes/moonmapper_autonomy/` |
| ~~`moonmapper_description` EKF/jsp/urdf-orphans~~ | Flyttet 2026-05-17 — se `arkiverte_koder/gamle_{urdf,config,meshes,models,scripts,dokumentasjon_utkast}/moonmapper_description/` |
| `colcon_build.sh` | Hjelpescript — beholdt |
| Referanser til `rover/` i `digital_tvilling.md` | Fysisk submodule fjernet tidligere; dokumentasjon peker til ekstern sti — vurder oppdatering ved innlevering |
| `ml/datasets/live/` | Live-testdata — beholdt for `predict.py` |
| `notebooks/` (tom) | Mappe kan fjernes manuelt eller beholdes |

## Validering

Kjørt etter opprydding (aktive pakker):

```bash
source /opt/ros/jazzy/setup.bash
cd moonmapper_ws
colcon build --symlink-install \
  --packages-select \
  moonmapper_autonomy moonmapper_bringup moonmapper_description \
  moonmapper_nav2
# Resultat: 10 packages finished
```

Etter arkivering av ROS-pakker: fjern **stale** `install/<pakke>` og `build/<pakke>` for arkiverte navn (ellers feiler `gz_sim.launch.py` med `package 'moonmapper_*' not found`):

```bash
for pkg in moonmapper_perception moonmapper_ml moonmapper_interfaces \
  moonmapper_gz_sensors moonmapper_msgs moonmapper_localization; do
  rm -rf install/$pkg build/$pkg
done
colcon build --symlink-install --packages-select \
  moonmapper_description moonmapper_bringup moonmapper_autonomy moonmapper_nav2
source install/setup.bash   # ny terminal eller re-source
```

Anbefalt funksjonstest:

```bash
source install/setup.bash

ros2 launch moonmapper_bringup sim_rover_clean.launch.py use_sim_time:=true

ros2 launch moonmapper_nav2 autonomous_exploration_full.launch.py \
  use_sim_time:=true world_preset:=earth_explore \
  start_sim:=true start_slam:=true start_nav2:=true start_explorer:=true \
  navigation_stack_delay:=12.0 explorer_extra_delay_sec:=5.0
```

```bash
ros2 run moonmapper_nav2 check_autonomous_exploration_stack.sh
```

## Gjenoppretting fra arkiv

```bash
# Eksempel: hent tilbake én launch
git mv arkiverte_koder/gamle_nav2_tester/launch/nav2_odom_baseline.launch.py \
  src/moonmapper_nav2/launch/
```

Se `arkiverte_koder/README.md`.
