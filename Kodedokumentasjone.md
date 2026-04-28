# Kodedokumentasjon (MoonMapper)

Dette dokumentet er ment som en praktisk “kartbok” over kodebasen: hva hver pakke gjør, hvordan den kjøres, og hvilke viktige parametre/argumenter som finnes.

## Filindeks (hele workspace)

Denne seksjonen er en “hvor ligger hva”-oversikt. Filene er gruppert per ROS-pakke i `src/`, og deretter “øvrige” filer på rot-/docs-nivå.

### ROS-pakker i `src/`

- **`src/moonmapper_bringup/`**
  - `package.xml`, `setup.py`, `setup.cfg`
  - `launch/sim.launch.py`
  - `launch/sensor_bringup.launch.py`
  - `config/moonmapper_rviz.rviz`
- **`src/moonmapper_control/`**
  - `package.xml`, `setup.py`, `setup.cfg`
  - `moonmapper_control/teleop_keyboard.py`, `moonmapper_control/__init__.py`
- **`src/moonmapper_description/`**
  - `package.xml`, `CMakeLists.txt`, `hooks/moonmapper_description.dsv.in`
  - `urdf/`:
    - `moonmapper_rover.urdf.xacro`
    - `moonmapper_rover_gazebo.urdf.xacro`
    - `moonmapper.urdf.xacro`
    - `materials.xacro`
    - `moonmapper_macros.xacro`
  - `launch/`:
    - `gazebo_rover.launch.py`
    - `sensor_bringup.launch.py` (via `moonmapper_bringup`)
    - `view_robot.launch.py`
    - `view_robot_with_diff.launch.py`
    - `rocker_diff_joint.py` (ROS 2 node: `rocker_bogie_kinematics`)
    - `unity_cmd_vel_node.py`
    - `unity_minimal.launch.py`
  - `config/`:
    - `jsp_drivers_only.yaml`
    - `wheel_controllers.yaml`
    - `ros_gz_bridge.yaml`
    - `unity_params.yaml`
  - `gz_plugins/src/`:
    - `RockerBogieDifferential.cc`
  - `worlds/`:
    - `moon_arena.sdf`
    - `rocker_bogie_test.sdf`
  - `rviz/moonmapper.rviz`
  - `meshes/` (inkl. `meshes/collision/.gitkeep`)
  - `scripts/`:
    - `archive_gazebo.sh`
    - `restore_gazebo.sh`
    - `export_urdf_for_unity.sh`
    - `convert_stl_to_dae.sh`
  - `unity_scripts/`:
    - `README.md`
    - `ArenaBoundary.cs`
    - `CameraImagePublisher.cs`
    - `ImuPublisher.cs`
    - `JointStatePublisher.cs`
    - `RoverSpawn.cs`
  - Dokumentasjon:
    - `Gazebo_test.md`
    - `Level1_Sensors.md`
    - `Unity_setup.md`
    - `Unity_arena_setup.md`
- **`src/moonmapper_gz_sensors/`**
  - `package.xml`, `CMakeLists.txt`
  - `gz_plugins/src/TriadSpectroscopy.cc`
  - `config/triad_material_map.yaml`
- **`src/moonmapper_localization/`**
  - `package.xml`, `CMakeLists.txt`
  - `config/ekf.yaml`
- **`src/moonmapper_msgs/`**
  - `package.xml`, `CMakeLists.txt`
  - `msg/TriadSpectrum.msg`
- **`src/moonmapper_navigation/`**
  - `package.xml`, `CMakeLists.txt`
  - `launch/nav2.launch.py`
  - `config/nav2_params.yaml`
  - `config/nav2_bt_nav_params.yaml`
- **`src/moonmapper_slam/`**
  - `package.xml`, `CMakeLists.txt`
  - `launch/slam.launch.py`
  - `config/slam_toolbox_params.yaml`
- **`src/moonmapper_webots/`**
  - `package.xml`, `setup.py`
  - `launch/webots_launch.py`
  - `moonmapper_webots/moonmapper_driver.py`, `moonmapper_webots/__init__.py`
  - `resource/moonmapper_rover_webots.urdf`
  - `worlds/moonmapper_world.wbt`
- **`src/ros_tcp_endpoint/`** (tredjepart/bridge mot Unity)
  - `package.xml`, `setup.py`, `setup.cfg`, `requirements.txt`
  - `launch/endpoint.py`
  - `ros_tcp_endpoint/`:
    - `server.py`, `client.py`, `unity_service.py`
    - `publisher.py`, `subscriber.py`
    - `communication.py`, `tcp_sender.py`, `thread_pauser.py`
    - `default_server_endpoint.py`, `exceptions.py`, `service.py`, `__init__.py`
  - Repo-meta: `README.md`, `LICENSE`, `CHANGELOG.md`, `CONTRIBUTING.md`, `CODE_OF_CONDUCT.md`, `.pre-commit-config.yaml`, `.github/*`, `.yamato/*`, `test/*`, `.gitignore`

### Øvrige filer i workspace-roten

- `Kodedokumentasjone.md` (denne fila)
- `docs/README.md`
- `docs/PROJECT_STRUCTURE.md`
- `docs/LEGACY_SCRIPTS.md`
- `Fysikk.md`
- `TEST.md`
- `frames_2026-04-22_19.20.43.gv`
- `unity_export/moonmapper.urdf`
- `.gitignore`
- `.gitmodules`

## Filtyper i denne kodebasen (hvorfor XML/YAML er viktige)

### XML (ROS 2 / Gazebo)

- **`package.xml`**: metadata + avhengigheter for ROS 2-pakker. Viktig for `rosdep`, `colcon`-byggerekkefølge, og at `ros2 launch` finner riktige runtime-avhengigheter.
- **URDF/Xacro (`*.urdf`, `*.urdf.xacro`)**: robotens kinematiske modell (link/joint-navn, geometri, inertial). Brukes av:
  - `robot_state_publisher` (TF)
  - RViz (RobotModel)
  - Gazebo (via “gazebo-xacro”/SDF-tagging for sensorer/plugins)
- **SDF/world (`*.sdf`)**: simulatorverden og modelloppsett i Gazebo (fysikk, lys, plugins, osv.).

### YAML (ROS 2)

YAML brukes nesten alltid til **parameter-konfig** for ROS 2 noder og stacks:
- **Controller/drivere** (f.eks. `ros2_control` controller-konfig)
- **Robot-estimering** (f.eks. EKF i `robot_localization`)
- **Nav2/SLAM** (mange tunables)
- **joint_state_publisher(_gui)** (hvilke joints som skal vises/ikke vises som slidere)

## `moonmapper_bringup` (ROS 2) – sim- og sensor-bringup

**Plassering**: `src/moonmapper_bringup/`  
**Formål**: Samler launch-filer og konfigurasjon for å starte MoonMapper-roboten i simulering, med valgfritt teleop/estimering/SLAM/Nav2 og RViz.

### Innhold (fil for fil)

#### `package.xml`

**Rolle**: ROS 2 pakke-manifest (navn, versjon, avhengigheter).  
**Viktig**:
- Pakken er en Python/launch-pakke (`ament_python`).
- Byggeverktøy: `ament_cmake_python` (typisk for “python package i colcon”).

**Avhengigheter** (utdrag, slik de brukes av launch-filene):
- **`moonmapper_description`**: URDF/Xacro for robotbeskrivelse (brukes for `robot_state_publisher` og Gazebo-launch-inkludering).
- **`moonmapper_webots`**: Webots launcher/driver (inkluderes i `sim.launch.py`).
- **`moonmapper_control`**: teleop-node (`teleop_keyboard`).
- **`moonmapper_localization`**: peker til EKF-konfig (`config/ekf.yaml`).
- **`moonmapper_navigation`**: Nav2 launch (inkluderes valgfritt).
- **`moonmapper_slam`**: SLAM launch (inkluderes valgfritt).
- **`robot_state_publisher`**: publiserer TF fra URDF.
- **`robot_localization`**: `ekf_node`.
- **`rviz2`**, **`xacro`**, samt `launch`/`launch_ros` som runtime.

#### `setup.py` / `setup.cfg`

**Rolle**: Installasjon/“packaging” for ROS 2 Python-pakka.

**Hva installeres som “share resources”**:
- Launch-filer:
  - `launch/sim.launch.py`
  - `launch/sensor_bringup.launch.py`
- RViz-konfig:
  - `config/moonmapper_rviz.rviz`
- `package.xml`

`setup.cfg` styrer hvor “scripts” installeres, men denne pakka har ingen `console_scripts`.

#### `launch/sim.launch.py` – Webots sim bringup

**Rolle**: Start “sim-stack” med Webots + nødvendige ROS-noder, og lar deg skru på/av typiske moduler.

**Starter alltid**:
- **Webots + driver** ved å inkludere `moonmapper_webots/launch/webots_launch.py`
- **`robot_state_publisher`** med `robot_description` generert fra Xacro:
  - Leser `moonmapper_description/urdf/moonmapper_rover.urdf.xacro`
  - Setter parameter `use_sim_time`

**Starter valgfritt** (styrt av launch-argumenter):
- **Teleop**: `moonmapper_control/teleop_keyboard`
- **EKF**: `robot_localization/ekf_node` med params fra `moonmapper_localization/config/ekf.yaml`
- **SLAM**: inkluderer `moonmapper_slam/launch/slam.launch.py`
- **Nav2**: inkluderer `moonmapper_navigation/launch/nav2.launch.py`
- **RViz2**: med config `config/moonmapper_rviz.rviz`

**Launch-argumenter** (med default):
- `use_rviz:=true|false` (default `true`)
- `use_teleop:=true|false` (default `false`)
- `use_ekf:=true|false` (default `false`)
- `ekf_publish_tf:=true|false` (default `true`)
  - Brukes for å unngå dobbelt-publisering av TF `odom -> base_link` hvis simulering/driver allerede gjør det.
- `use_slam:=true|false` (default `false`)
- `use_nav2:=true|false` (default `false`)
- `use_sim_time:=true|false` (default `true`)

**Kjøring**:

```bash
ros2 launch moonmapper_bringup sim.launch.py
```

Eksempel med flere moduler:

```bash
ros2 launch moonmapper_bringup sim.launch.py use_teleop:=true use_ekf:=true use_rviz:=true
```

#### `launch/sensor_bringup.launch.py` – Gazebo sensor bringup (wrapper)

**Rolle**: “Level 1” sensor-bringup for Gazebo Sim ved å inkludere en annen launch-fil som gjør den faktiske Gazebo-oppsettingen.

**Hva betyr “wrapper” her?**  
En *wrapper* er et tynt oppstarts-lag som **samler og videresender argumenter** til en under-launch (i dette tilfellet `moonmapper_description/launch/gazebo_rover.launch.py`). Det gir én enkel inngang (`sensor_bringup.launch.py`) i stedet for at brukeren må huske alle detaljer.

**Launch-argumenter** (med default):
- `world`: default `moonmapper_description/worlds/moon_arena.sdf`
- `use_rviz:=true|false` (default `true`)
- `spawn_z`: default `0.15`
  - Kommentar i fila indikerer at roveren er liten og at en “trygg” z ved spawn reduserer sjansen for uønsket starttilstand i fysikken.
- `enable_diff_plugin:=true|false` (default `true`)

**Hva som faktisk skjer**:
- Inkluderer `moonmapper_description/launch/gazebo_rover.launch.py`
- Sender videre argumentene `world`, `use_rviz`, `spawn_z`, `enable_diff_plugin`

**Kjøring**:

```bash
ros2 launch moonmapper_bringup sensor_bringup.launch.py
```

Med egen verden:

```bash
ros2 launch moonmapper_bringup sensor_bringup.launch.py world:=/abs/path/to/world.sdf
```

#### `config/moonmapper_rviz.rviz` – RViz-oppsett

**Rolle**: Standard RViz-visning for MoonMapper.

**Viktige valg i config**:
- Fixed Frame: **`odom`**
- Displays:
  - **TF** (på)
  - **RobotModel** (på)
  - **LaserScan** på topic **`/scan`** (på)
  - **Map** på topic **`/map`** (av som default)

### Typiske “stacks” (mentalt kart)

- **Webots-sim**: `moonmapper_bringup/sim.launch.py`
  - Webots + driver
  - TF via `robot_state_publisher`
  - (valgfritt) teleop, EKF, SLAM, Nav2, RViz
- **Gazebo sensor bringup**: `moonmapper_bringup/sensor_bringup.launch.py`
  - “Wrapper” som peker til `moonmapper_description` sin Gazebo-launch

## `moonmapper_description` (ROS 2) – robotmodell, launch og Gazebo-plugins

**Plassering**: `src/moonmapper_description/`  
**Formål**: Inneholder robotbeskrivelsen (URDF/Xacro), konfigfiler, launch-filer og Gazebo-spesifikke system-plugins.

### Viktig navnekontrakt: joint-navn må matche URDF

I ROS/Gazebo er joint-navn en “API-kontrakt”: YAML/launch/noder/plugins refererer til joint-navn som strenger, og de må finnes i URDF/Xacro.

I denne repo-versjonen finnes bl.a. disse jointene i `moonmapper_description/urdf/moonmapper_rover.urdf.xacro`:
- `rocker_left_joint`, `rocker_right_joint`
- `bogie_left_joint`, `bogie_right_joint`
- `rocker_bogie_diff_joint`

**Merk**:
- `hengsel_diff_L_joint` / `hengsel_diff_R_joint` finnes **ikke** i denne Xacroen.
- Derfor er “hinge joints” gjort **valgfrie** i både RViz-kinematikk-noden og Gazebo-pluginen (default tom streng = ignorert).

### `config/jsp_drivers_only.yaml` – “drivers only” i joint_state_publisher_gui

**Rolle**: Parameterfil for `joint_state_publisher` og `joint_state_publisher_gui` som skjuler avhengige joints slik at GUI bare viser de du faktisk skal styre.

**Hva den gjør**:
- Lar kun `rocker_left_joint` og `rocker_right_joint` være manipulerbare slidere.
- Markerer `bogie_left_joint`, `bogie_right_joint` og `rocker_bogie_diff_joint` som `dependent_joints` slik at GUI ikke rendrer sliders for dem.

### `launch/view_robot_with_diff.launch.py` – RViz/GUI visualisering med kinematikk

Denne launch-fila setter opp en ren visualiserings-pipeline:
- `joint_state_publisher_gui` publiserer til `/joint_states_raw` (via remap)
- `rocker_bogie_kinematics` fyller inn/overskriver avhengige joints og publiserer `/joint_states`
- `robot_state_publisher` lager TF fra `/joint_states`
- `rviz2` viser modellen

### `launch/rocker_diff_joint.py` – `rocker_bogie_kinematics` (ROS 2 node)

**Rolle**: ROS 2 node som gjør kinematisk kobling for RViz ved å gjøre `/joint_states_raw` om til en komplett `/joint_states`.

**Input/Output**:
- Input: `input_topic` (default `/joint_states_raw`)
- Output: `output_topic` (default `/joint_states`)

**Hva den regner ut**:
- Diff-ledd: \(q_{diff} = k_{diff\_L} L + k_{diff\_R} R + k_{diff\_0}\)
- Bogie-ledd: \(q_{bogie} = a\cdot rocker + b\cdot q_{diff} + c\)
- Hengsel-ledd (valgfritt): \(q_{hinge} = d\cdot rocker + e\)

**Joint-navn (defaults)**:
- rockere: `rocker_left_joint`, `rocker_right_joint`
- bogies: `bogie_left_joint`, `bogie_right_joint`
- diff: `rocker_bogie_diff_joint`
- hengsler: `""` (disabled) med mindre de settes via launch

### `gz_plugins/src/RockerBogieDifferential.cc` – Gazebo system-plugin (fysikk)

**Rolle**: Gazebo Sim system-plugin som kjører i simulatorens PreUpdate-loop og påvirker fysikken direkte.

**Den gjør to ting**:
- **1) Walking-beam / sum-constraint på rockerne**: legger moment på `rocker_left_joint` og `rocker_right_joint` slik at \(q_L + s q_R\) drives mot en `target_sum` (PD-kontroll). `right_sign` (= \(s\)) brukes hvis høyre joint har motsatt fortegn.
- **2) PD-drive av “kosmetiske” ledd**:
  - `rocker_bogie_diff_joint` drives mot en målposisjon \(q_{diff}^*\)
  - `hengsel_left_joint` / `hengsel_right_joint` er **valgfrie** (default `""`) og ignoreres hvis de ikke finnes i URDF

