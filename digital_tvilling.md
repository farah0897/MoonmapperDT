# Digital tvilling — MoonMapper (kapittel 6)

> **Formål:** Kildegrunnlag for `\chapter{Implementasjon av digital tvilling}` (kap. 6 i rapportmalen).  
> **Full rapportstruktur** (kap. 1–15, vedlegg, figurer): se [`bachelor_rapport_utkast.md`](bachelor_rapport_utkast.md).  
> Teksten kan kopieres til LaTeX med mindre tilpasning (figurhenvisninger, `\ref{appendix:system_arkitektur}`, osv.).

---

## Innledning

Ved siden av den fysiske MoonMapper-roveren er det utviklet en **digital tvilling** i ROS 2-workspacet `moonmapper_ws`. Tvillingen skal etterligne den fysiske roboten tilstrekkelig nært til at samme autonomi-stack (SLAM, Nav2, frontier-utforskning, sikkerhetslag) kan kjøres i simulasjon før og parallelt med feltforsøk.

**Hovedprinsipp:** Én felles **robotbeskrivelse** (URDF/Xacro) definerer geometri, masse, ledd og sensor-rammer. Gazebo Sim 8 (Harmonic) utvider denne med fysikk, sensor-plugins og `ros2_control`. Topic-navn og TF-rammer speiles der det trengs mot den fysiske roveren (f.eks. RealSense D435 via `camera_aliases`).

**Simulatormotor:** Primær digital tvilling er **Gazebo Sim 8** med DART-fysikk. Et alternativ finnes i pakken `moonmapper_webots` (`sim.launch.py`), men hovedstacken for autonom utforskning bruker Gazebo.

Diagram i vedlegg (systemarkitektur, `\ref{appendix:system_arkitektur}`) viser kommunikasjon mellom systemets komponenter: kommandoer (`cmd_vel`-kjeden), sensordata (broer Gazebo → ROS), odometri/TF, logging og Nav2/RTAB-Map. Nedenfor er samme informasjon utdypet per implementasjonslag.

### Overordnet dataflyt (tekstlig)

```
┌─────────────────┐     ros_gz_bridge      ┌──────────────────┐
│  Gazebo Sim 8   │ ──────────────────────►│  ROS 2 (topics)  │
│  (fysikk,       │   clock, IMU, kamera,  │  robot_state_    │
│   sensorer)     │   depth, joint_states  │  publisher, …    │
└────────┬────────┘                        └────────┬─────────┘
         │ gz_ros2_control                           │
         │ diff_drive + hjul                         │ /camera/… (alias)
         ▼                                           ▼
    /diff_drive_controller/odom              RTAB-Map, Nav2, frontier
         │                                           │
         └──────────────► /odom, TF odom→base_footprint
```

---

## Implementasjon

### Programvare-stack

| Lag | Pakke / fil | Rolle |
|-----|-------------|--------|
| Robotmodell | `moonmapper_description` | URDF/Xacro, mesh, Gazebo-utvidelse, verdener |
| Sim-bringup | `moonmapper_bringup` | `gazebo_rover`, `cmd_vel_odom_relay`, `camera_aliases` |
| Gazebo-plugins | `moonmapper_gz_sensors`, `gz_plugins` | Rocker-bogie-differensial, syntetisk triad-spektroskopi |
| Autonomi | `moonmapper_nav2`, `moonmapper_autonomy` | Nav2, depth→scan, sikkerhet, frontier explorer |
| Fysisk referanse | `rover/code/dynamixel_driver.py` | `/cmd_vel`, `/odom`, Dynamixel 6WD |

**Bygg og kjøring (autonom utforskning i sim):**

```bash
cd moonmapper_ws
colcon build --symlink-install
source install/setup.bash

ros2 launch moonmapper_nav2 autonomous_exploration_full.launch.py \
  use_sim_time:=true world_preset:=earth_explore \
  start_sim:=true start_slam:=true start_nav2:=true start_explorer:=true \
  navigation_stack_delay:=12.0 explorer_extra_delay_sec:=5.0
```

**Minimal sim (kun robot + sensorer):**

```bash
ros2 launch moonmapper_bringup sim_rover_clean.launch.py use_sim_time:=true
```

### Parallell til fysisk robot

`rover/simulation/Simulation Guide.md` beskriver kontrakten: simulasjonen skal erstatte Step 1–4 i oppstartsguiden slik at Step 5–6 (RTAB-Map, Nav2) kan kjøres uendret.

| Fysisk | Sim (Gazebo) |
|--------|----------------|
| Dynamixel 6WD, `/cmd_vel` | `diff_drive_controller` via `cmd_vel_odom_relay` |
| Wheel odometry `/odom` | `/diff_drive_controller/odom` → speilet `/odom` |
| RealSense D435 topics | `/depth_camera/*` → `camera_aliases` → `/camera/camera/…` |
| `odom → base_link → camera_*` TF | `odom → base_footprint → base_link → …` (REP-105) |

Fysiske hjulparametre i driver: `WHEEL_RADIUS_METERS = 0.05`, `WHEEL_SEPARATION_METERS = 0.29`. I sim brukes målte verdier fra URDF: radius **0,045 m**, sporvidde **0,217 m** (`diff_drive_controller.yaml`) — bevisst kalibrert mot modellen, ikke kopiert blindt fra driver.

---

## Robotmodell og 3D-modell

### Opprinnelse og omfang

Robotgeometrien er eksportert fra CAD (STL) og organisert under:

`src/moonmapper_description/meshes/visual/`

Viktige mesh-filer:

- `robot_base.stl` — chassis (`base_link`)
- `rocker_*_link.stl`, `bogie_*_link.stl` — rocker-bogie-suspensjon
- `wheel_l{1,2,3}_link.stl`, `wheel_r{1,2,3}_link.stl` — seks hjul
- `steriokamera_link.stl`, `Mikroskop_link.stl`, `triadsensor{1,2}_link.stl`, `IMU_link.stl`

Kollisjonsgeometri bruker enten samme STL (rocker) eller **primitive** (hjul som sylinder r=0,045 m, bredde 0,039 m) for stabil kontakt i Gazebo. Profilen `simple_collision_debug` fjerner unødvendig chassis/rocker-kollisjon og anbefales for 6WD-traksjon.

### Mekanisk layout (kort)

- **6 hjul**, kontinuerlige ledd (`wheel_l1..l3`, `wheel_r1..r3`), akse langs **Y** i hjul-link.
- **Rocker-bogie** med revolute rockere og bogier; lukket push-rod-mekanisme modellert med prisme+mimic i RViz, **fixed** stag i Gazebo + system-plugin for differensial.
- **base_footprint** ligger i hjulplan (z=0); `base_link` er **0,1242 m** over footprint.
- **+X** er mot front (stereo/depth montert foran).

Masse og treghet på `base_link` er målt/estimert (ca. **0,907 kg**) med eksplisitt advarsel i URDF om at inertial-offset ikke flyttes bakover for å «løfte nesa».

---

## URDF/Xacro/SDF-struktur

### Filhierarki

```
moonmapper.urdf.xacro          # Entry point (inkluderer rover)
    └── moonmapper_rover.urdf.xacro   # Full kinematik + sensor-links
            └── (materials, macros)

moonmapper_rover_gazebo.urdf.xacro  # Inkluderer rover + Gazebo-tags
    ├── wheel_gazebo (friksjon)
    ├── gz_ros2_control plugin
    ├── RockerBogieDifferential plugin
    ├── sensor <sensor> på base_link
    └── TriadSpectroscopy (valgfritt)
```

**Launch** prosesserer typisk `moonmapper_rover_gazebo.urdf.xacro` med `use_gazebo:=true` og fysikk-argumenter (`wheel_mu1`, `physics_profile`, osv.).

### Viktige xacro-argumenter (`moonmapper_rover.urdf.xacro`)

| Argument | Standard | Betydning |
|----------|----------|-----------|
| `use_gazebo` | `false` | Fixed push-rod i sim; prisme av |
| `enable_urdf_mimic` | `true` | Mimic for RViz/visualisering |
| `simple_collision_debug` | `false` | Kun hjul-kollisjon |

### Ledd (DOF-oversikt)

| Joint | Type | Funksjon |
|-------|------|----------|
| `rocker_left_joint`, `rocker_right_joint` | revolute | Rocker-pivot |
| `bogie_left_joint`, `bogie_right_joint` | revolute | Bogie på rocker |
| `wheel_*_joint` | continuous | Hjul (styrt av diff_drive) |
| `rocker_bogie_diff_joint` | revolute | Differensialstav (passiv + plugin) |
| Push-rod (`stang_*`, `feste_mellom_*`) | prismatic/fixed | RViz mimic vs Gazebo fixed |

### Sensor-links (TF)

Faste ledd fra `base_link`:

- `imu_link`
- `stereo_camera_link` → venstre/høyre kamera + `*_optical_frame`
- `depth_camera_link` → `depth_camera_optical_frame`
- `mikroskop_link` → `mikroskop_optical_frame`
- `triad_sensor_1_link`, `triad_sensor_2_link`

### SDF-verdener

- **Verdener:** `worlds/*.sdf` — frittstående Gazebo-verdener (ikke generert fra xacro).
- **Innebygd modell:** `models/replan_test_block/model.sdf` — testhindring.
- Roveren spawnes fra URDF via `ros_gz_sim create`, ikke som egen SDF-modell.

### Gazebo-spesifikke utvidelser

1. **`gz_ros2_control`** — `wheel_controllers.yaml` + spawnet `joint_state_broadcaster` og `diff_drive_controller`.
2. **`RockerBogieDifferential`** (`libmoonmapper_rocker_bogie_differential.so`) — walking-beam PD + diff-stav; modus `weak` (standard) / `full` / `off`.
3. **Sensorer** festet på `base_link` med `<pose>` som **må** matche URDF TF (kommentert i xacro).

---

## Gazebo-verden og scenarioer

### Verdensfiler

| `world_preset` | SDF-fil | Beskrivelse |
|----------------|---------|-------------|
| `moon` | `moon_arena.sdf` | 20×20 m jordaktig flate, hjørnestolper, ingen innvendige murer/hindringer |
| `earth` | `earth_arena.sdf` | Jord-scenario (stabil profil `earth_stable_6wd`) |
| `earth_explore` | `earth_arena_explore.sdf` | **3×6 m** innhegnet boks i 20×20 m plan — brukt til autonom utforskning |

Felles for alle:

- Fysikk: **DART** (`dartsim`), tidssteg 1 ms, `use_sim_time` via `/clock`.
- System-plugins: physics, user commands, scene broadcaster, contact, **sensors** (ogre2), IMU.
- Tyngdeakselerasjon **−9,81 m/s²** (jord, også i «moon_arena»-navnet etter konvertering fra månemiljø).

### Fysikkprofiler (`gazebo_rover.launch.py`)

Launch-argument `physics_profile` (eller auto fra `world_preset`) styrer hjulfriksjon (`wheel_mu1`, `wheel_mu2`), kontakt `kp`/`kd`, `spawn_z` og `simple_collision_debug`. Eksempler: `safe_6wd`, `earth_stable_6wd`, `realistic_6wd`, `debug_low_friction`.

### Scenario i oppgaven

For bachelor/demo av autonom utforskning er **`earth_explore`** valgt: avgrenset arena som gir forutsigbare frontiers og kartstørrelse, samtidig som RTAB-Map får visuelle holdepunkter (hjørner, vegger).

---

## Sensorimplementasjon

### Bro: Gazebo Transport → ROS 2

`config/ros_gz_bridge.yaml` brukes av `ros_gz_bridge parameter_bridge`.

| ROS-topic | GZ-topic | Type |
|-----------|----------|------|
| `/clock` | `/clock` | sim-klokke |
| `/imu/data` | `/imu` | IMU 100 Hz |
| `/stereo/left|right/image_raw` + `camera_info` | samme navn | RGB 320×240, 15 Hz |
| `/depth_camera/image`, `depth_image`, `points`, `camera_info` | depth_camera | RGB-D |
| `/depth/image_raw`, … | alias til depth_camera | Level-1-navn |
| `/microscope/image_raw`, `camera_info` | mikroskop | inspeksjonskamera |

**Bevisst utelatt fra bro:**

- `/cmd_vel` til Gazebo (konflikt med `ros2_control`).
- Modellpose → `/tf` (duplikat TF-trær; odometri kommer fra diff_drive).

### Sensorer i URDF/Gazebo (`moonmapper_rover_gazebo.urdf.xacro`)

| Sensor | GZ-type | Rate | Merknad |
|--------|---------|------|---------|
| IMU | `imu` | 100 Hz | Støy på accel/gyro |
| Stereo L/R | `camera` | 15 Hz | 70° FOV, `gz_frame_id` = optical frame |
| Depth | `rgbd_camera` | 15 Hz | RGB + depth + point cloud; klipp 0,05–10 m |
| Mikroskop | `camera` | 10 Hz | Nedover pekende |
| Triad 1/2 | custom plugin | 20 Hz | Raycast-spektrum mot material_map |

`/scan` for Nav2 kommer **ikke** fra Gazebo-lidar, men fra `moonmapper_autonomy/depth_to_scan_node` som projiserer dybdekamera til `sensor_msgs/LaserScan`.

### Alias mot fysisk RealSense

`moonmapper_bringup/camera_aliases.py` republiserer:

```
/depth_camera/image        → /camera/camera/color/image_raw
/depth_camera/depth_image  → /camera/camera/depth/image_rect_raw
/depth_camera/camera_info  → /camera/camera/color/camera_info
/depth_camera/points       → /camera/camera/depth/color/points
```

med `camera_color_optical_frame` / `camera_depth_optical_frame`. Statisk TF for `camera_link` og optical frames settes i `gazebo_rover.launch.py` / bringup.

### Validering

Se `src/moonmapper_description/Level1_Sensors.md` for sjekkliste (`ros2 topic hz`, `view_frames`).

---

## Launch-system og TF-struktur

### Launch-hierarki (Gazebo)

```
autonomous_exploration_full.launch.py   [moonmapper_nav2]
    ├── sim_rover_clean.launch.py       [moonmapper_bringup]
    │       └── gazebo_rover.launch.py  [moonmapper_description]
    ├── (forsinket) rtabmap_sim.launch.py
    ├── (forsinket) nav2_rtabmap_navigation.launch.py
    └── (forsinket) frontier_explorer
```

`gazebo_rover.launch.py` starter bl.a.:

1. `gz sim` med valgt `.sdf`
2. `robot_state_publisher` (xacro → URDF)
3. `ros_gz_sim create` (spawn)
4. `ros_gz_bridge` (sensorer + clock)
5. Controller-spawners (`joint_state_broadcaster`, `diff_drive_controller`)
6. `cmd_vel_odom_relay`, ev. `camera_aliases`, EKF (valgfritt)

### TF-trær (sim, typisk autonom kjøring)

```
map  ──(RTAB-Map / AMCL-lignende)──►  odom
odom ──(diff_drive_controller)──────►  base_footprint
base_footprint ──(URDF fixed)───────►  base_link
base_link ──(URDF)─────────────────►  imu_link, stereo_*, depth_*, mikroskop_*, …
```

Statiske alias (sim): `camera_link`, `camera_color_optical_frame`, `camera_depth_optical_frame` kobles til depth-stereo-kjeden for kompatibilitet med fysisk stack.

**Viktig:** Kun **én** kilde skal publisere `odom → base_footprint` (diff_drive). Ikke bro Gazebo-modellpose til `/tf` samtidig.

### `use_sim_time`

Alle noder i sim skal bruke `use_sim_time:=true` og motta `/clock` fra broen — ellers feiler tidsstempler (f.eks. collision monitor, `camera_aliases` med `refresh_header_stamp`).

### Tidsforsinkelinger (full autonomi)

| Fase | Standard delay | Formål |
|------|----------------|--------|
| SLAM (RTAB-Map) | 6 s etter launch | Fysikk stabilisert etter spawn |
| Nav2 | 12 s | Kart/TF klart |
| Frontier explorer | +5 s | Nav2 lifecycle ferdig |

---

## Bevegelse og `cmd_vel`-kjede

### Sim: fra Nav2 til hjul

```
Nav2 controller_server
    → cmd_vel_smoothed
    → collision_monitor (pass-through, polygoner av)
    → /cmd_vel_raw
    → safety_obstacle_node (scan-gating)
    → /cmd_vel
    → cmd_vel_odom_relay  (Twist → TwistStamped)
    → /diff_drive_controller/cmd_vel
    → gz_ros2_control (JointVelocityCmd på 6 hjul)
    → Gazebo DART-kontakt
    → /joint_states, /diff_drive_controller/odom, TF odom→base_footprint
    → (relay) /odom
```

Konfigurasjon: `nav2_params_rtabmap_sim.yaml` (`cmd_vel_in_topic: cmd_vel_smoothed`, `cmd_vel_out_topic: cmd_vel_raw`).

**Teleop / test uten Nav2:**

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -r /cmd_vel:=/diff_drive_controller/cmd_vel -p stamped:=true -p frame_id:=base_footprint
```

eller direkte:

```bash
ros2 topic pub --once /diff_drive_controller/cmd_vel geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: base_footprint}, twist: {linear: {x: 0.08}, angular: {z: 0.0}}}"
```

### Fysisk robot

`rover/code/dynamixel_driver.py`:

- Abonnerer på `/cmd_vel` (`geometry_msgs/Twist`)
- Setter hastighet på Dynamixel ID 1–6 (venstre 1,3,5 — høyre 2,4,6)
- Integrerer encodere → `/odom` + `odom → base_link` TF

### `cmd_vel_odom_relay`

Node i `moonmapper_bringup`:

- Konverterer `/cmd_vel` (Twist) til `/diff_drive_controller/cmd_vel` (TwistStamped) — **påkrevd** på ROS 2 Jazzy / `ros2_controllers` v4.x.
- Speiler `/diff_drive_controller/odom` → `/odom` når `publish_odom_relay:=true`.
- Parametre: `frame_id` (default `base_footprint`), `cmd_linear_x_sign`, `cmd_angular_z_sign`.

### Diff-drive-parametre (sim)

Fra `config/diff_drive_controller.yaml`:

- Hjulradius: **0,045 m**
- Sporvidde: **0,217112 m**
- `max_velocity` lineær ±0,30 m/s, vinkel ±1,0 rad/s
- Publiserer TF `odom` → `base_footprint`

### Designvalg: ingen `/cmd_vel`-bro til Gazebo

`ros_gz_bridge.yaml` dokumenterer at parallell `ROS_TO_GZ` på `/cmd_vel` konkurrerer med `ros2_control` og gir symptomer som «hjul spinner, kropp snur ikke». Motorveien går **kun** via diff_drive-controller.

---

## Referanser i repo

| Dokument / fil | Innhold |
|----------------|---------|
| `README.md` | Kort sim-start |
| `Kodedokumentasjone.md` | Pakkeoversikt, rocker-plugin |
| `rover/simulation/Simulation Guide.md` | Kontrakt sim ↔ fysisk |
| `src/moonmapper_description/launch/gazebo_rover.launch.py` | Docstring, motorvei |
| `src/moonmapper_description/Level1_Sensors.md` | Sensor-sjekkliste |
| `src/moonmapper_autonomy/README.md` | depth→scan + safety-kjede |

---

## Forslag til LaTeX-integrasjon

- Kapittel: `\chapter{Digital tvilling}` med `\label{ch:digital-twin}`
- Seksjoner som i oppgavemalen; figurer: RViz/Gazebo-skjermbilder, `view_frames`-graf, systemarkitektur (vedlegg)
- Tabeller kan flyttes fra denne filen med `booktabs`
- Kodereferanser: filstier som `\texttt{moonmapper\_description/...}`

*Generert fra `moonmapper_ws`-kodebase, mai 2026.*
