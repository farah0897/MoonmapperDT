## MoonMapper – ROS 2 Jazzy rover simulation workspace (Webots + Gazebo Sim + Unity integration)

MoonMapper er et ROS 2 Jazzy workspace som inneholder:

- en **robotbeskrivelse** (URDF/Xacro) av en liten **6-hjuls rover** med **rocker–bogie**-inspirert kinematikk og et “walking-beam / differential”-ledd
- en **Webots-basert digital twin** (world + driver) med `/cmd_vel` → hjulmotorer, og publikasjon av `/odom` + TF
- en **Gazebo Sim (Harmonic / gz-sim8) stack** med:
  - `ros2_control` (diff_drive_controller over 6 hjul)
  - sensorer (IMU, stereo, RGB-D, mikroskop)
  - en egen Gazebo system-plugin for “rocker-bogie differential” og en egen Level 1 “Triad spectroscopy” sensor-plugin
- en **Unity-integrasjon** via `ros_tcp_endpoint` + ROS-side bro for `/cmd_vel` → 6 hjulhastigheter

Dette repoet ser ut til å være et student-/prosjektworkspace som skal støtte simulering, sensorer og grunnleggende autonomy-stacks (SLAM + Nav2). Det finnes **ikke** en full perception/ML pipeline i denne koden (utover synthetic “Triad spectroscopy” sensor).

> **Merk**: Workspace-rota (`moonmapper_ws/`) er ikke et git-repo i denne checkouten (ingen `.git` her). Det påvirker hvordan man sporer endringer, men ikke hvordan ROS-pakkene bygges.

---

## 1. Formål og målsetting

Basert på faktisk kode/filer i workspace er målbildet:

- **Simulere en rover** (6WD) med et rocker/bogie-oppsett og et mekanisk “differential”-ledd (walking beam).
- **Kjøre ROS 2-stacks** i sim:
  - teleop (`moonmapper_control`)
  - odometri + TF (Webots-driver, og/eller `diff_drive_controller` i Gazebo)
  - valgfri EKF (`robot_localization`)
  - valgfri SLAM (`slam_toolbox`)
  - valgfri Nav2 (konfig + launch)
- **Simulere sensorsuite** i Gazebo:
  - IMU + kameraer + RGB-D + mikroskop
  - “Triad spectroscopy” som en syntetisk Level 1 sensor (raycast + material map → spektrumvektor)

Work in progress / delvis:

- Unity-stack har tydelig integrasjon og scripts, men det er vanskelig å verifisere Unity-sceneoppsett kun fra repoet (må testes i Unity).
- Webots-driver er en enkel skid-steer modell med odometri integrert fra `/cmd_vel` (bevisst “sim-only”).

---

## 2. Hovedfunksjoner (implementert i repoet)

- **Robotmodell (URDF/Xacro)**:
  - detaljert link/joint-struktur for rover med rocker/bogie + sensorframes
  - bruk av `package://moonmapper_description/meshes/...` for visuelle/collision meshes
- **Rocker–bogie “differential”-mekanikk**:
  - RViz/visualisering: kinematikk-node som genererer avhengige joints fra rocker-joints
  - Gazebo: egen system-plugin (`moonmapper_rocker_bogie_differential`) som håndhever constraint via PD-moment
- **Webots simulation stack**:
  - world (`moonmapper_world.wbt`)
  - driver plugin (`moonmapper_driver.py`) som abonnerer på `/cmd_vel` og publiserer `/odom` + TF `odom->base_link`
- **Gazebo Sim (gz-sim8) simulation stack**:
  - launch som starter gz sim + spawner rover + `ros_gz_bridge` + `ros2_control` spawners + RViz (valgfritt)
  - `wheel_controllers.yaml` for `diff_drive_controller` over 6 hjul
  - sensor-konfig i Gazebo-xacro wrapper (`moonmapper_rover_gazebo.urdf.xacro`)
  - `ros_gz_bridge.yaml` som bro’er `/clock`, `/tf`, og sensor topics
- **Unity-integrasjon**:
  - `ros_tcp_endpoint` pakke (vendored)
  - `unity_cmd_vel_node.py` som publiserer `/unity/wheel_velocities` basert på `/cmd_vel`
  - Unity C# scripts i `moonmapper_description/unity_scripts/` (publisering av sensor- og joint data ser ut til å være forventet)
- **Navigation/SLAM/Localization**:
  - EKF config (`moonmapper_localization/config/ekf.yaml`)
  - SLAM launch + params (`moonmapper_slam`)
  - Nav2 launch + params (`moonmapper_navigation`)
- **Custom messages + synthetic sensors**:
  - `moonmapper_msgs/msg/TriadSpectrum.msg`
  - `moonmapper_gz_sensors` plugin som publiserer `TriadSpectrum` basert på raycast + YAML material mapping

---

## 3. Teknologistakk (basert på repoet)

- **ROS 2 Jazzy**: workspace er skrevet for Jazzy (packages/README/launch/dep-lister).
- **Python (rclpy / launch)**:
  - launch-filer og noder (teleop, unity cmd_vel bridge, rocker-bogie kinematics)
  - Webots-driver plugin
- **C++ (gz-sim8 system plugins)**:
  - `moonmapper_description/gz_plugins`: rocker-bogie differential plugin (bygges hvis gz-sim8 finnes)
  - `moonmapper_gz_sensors/gz_plugins`: Triad spectroscopy plugin
- **Webots**:
  - world (`.wbt`) og `webots_ros2_driver` launcher/controller
- **Gazebo Sim (Harmonic / gz-sim8) + ros_gz**:
  - `ros_gz_sim` spawn/create + `ros_gz_bridge` + `gz_ros2_control`
- **URDF/Xacro**:
  - robotbeskrivelse i `moonmapper_description/urdf/`
- **RViz2**:
  - visualisering for både Webots- og Gazebo-stack, pluss Unity-stack
- **Nav2 / slam_toolbox / robot_localization** (valgfritt):
  - launch/config inkludert i egne pakker

Ikke observert i repoet (bør verifiseres manuelt før man dokumenterer som feature):
- Docker, OpenCV/PCL, ML-rammeverk (PyTorch/TensorFlow), egen perception-pipeline

---

## 4. Repository-struktur (viktig)

Workspace (forkortet):

```text
moonmapper_ws/
├── src/                         # ROS 2 packages (source)
│   ├── moonmapper_bringup/      # Entry-point launch files
│   ├── moonmapper_control/      # teleop node (cmd_vel)
│   ├── moonmapper_description/  # URDF/Xacro + meshes + Gazebo + Unity integration
│   ├── moonmapper_gz_sensors/   # Gazebo Sim 8 Level 1 synthetic sensors (C++)
│   ├── moonmapper_localization/ # EKF config (robot_localization)
│   ├── moonmapper_msgs/         # custom message interfaces
│   ├── moonmapper_navigation/   # Nav2 config + launch
│   ├── moonmapper_slam/         # slam_toolbox config + launch
│   ├── moonmapper_webots/       # Webots world + driver plugin
│   └── ros_tcp_endpoint/        # Unity ROS-TCP endpoint (vendored)
├── docs/                        # Workspace documentation
├── meshes/                      # Root-level raw mesh exports (non-canonical; see docs)
├── unity_export/                # URDF exported for Unity pipelines (artifact)
├── build/ install/ log/         # colcon artifacts (not source)
└── README.md                    # This file
```

### Viktige mapper og hvordan de henger sammen

- **`src/moonmapper_bringup/`**: “hovedinnganger” for å starte systemet
  - `launch/sim.launch.py`: Webots bringup (driver + RSP + optional EKF/SLAM/Nav2/RViz)
  - `launch/sensor_bringup.launch.py`: Gazebo Level 1 sensors bringup (inkluderer Gazebo launch fra description)

- **`src/moonmapper_description/`**: canonical robotbeskrivelse + sim-integrasjon
  - `urdf/`: base + Gazebo wrapper Xacro
  - `meshes/`: runtime meshes (visual + collision)
  - `config/`: controllers, bridges, unity params
  - `launch/`: Gazebo, RViz view, Unity stacks, kinematics/bridge noder
  - `gz_plugins/`: C++ plugin (rocker-bogie differential)
  - `worlds/`: Gazebo worlds

- **`src/moonmapper_webots/`**: Webots world + driver plugin

- **`src/moonmapper_gz_sensors/`**: Level 1 synthetic sensor plugin + YAML material map

- **`src/ros_tcp_endpoint/`**: ROS-TCP endpoint for Unity integration

Se også:
- `docs/PROJECT_STRUCTURE.md`
- `docs/LEGACY_SCRIPTS.md`

---

## 5. Arkitektur / systemoversikt

### A) Webots stack (primær “MVP”)

1. `/cmd_vel` (Twist) → `moonmapper_webots/moonmapper_driver.py`
2. Driver setter 6 motorer + publiserer:
   - `/odom` (Odometry)
   - TF `odom -> base_link`
3. `robot_state_publisher` publiserer øvrige TF fra URDF.
4. RViz visualiserer robot og data.

### B) Gazebo Sim stack (Harmonic / gz-sim8)

1. `gazebo_rover.launch.py` starter `ros_gz_sim` + spawner rover fra `robot_description`.
2. `gz_ros2_control` leser `wheel_controllers.yaml` og aktiverer:
   - `joint_state_broadcaster`
   - `diff_drive_controller` (6 hjul)
3. `ros_gz_bridge` bruker `ros_gz_bridge.yaml` for `/clock`, `/tf` og sensor topics.
4. Gazebo wrapper Xacro:
   - laster plugins (differential + triad spectroscopy)
   - definerer IMU/stereo/RGB-D/mikroskop sensorer

### C) Unity stack (ROS-TCP)

1. `unity_minimal.launch.py`: `ros_tcp_endpoint` + `robot_state_publisher` (+ RViz)
2. `unity_full.launch.py`: + `/cmd_vel` → `/unity/wheel_velocities` via `unity_cmd_vel_node.py`

---

## 6. Fil-for-fil: de viktigste delene

### Launch

- `src/moonmapper_bringup/launch/sim.launch.py`: Webots bringup, optional EKF/SLAM/Nav2/RViz
- `src/moonmapper_bringup/launch/sensor_bringup.launch.py`: Gazebo Level 1 sensors bringup
- `src/moonmapper_description/launch/gazebo_rover.launch.py`: full Gazebo stack
- `src/moonmapper_description/launch/view_robot.launch.py`: URDF i RViz (JSP GUI)
- `src/moonmapper_description/launch/view_robot_with_diff.launch.py`: URDF i RViz + kinematic coupling via node
- `src/moonmapper_description/launch/unity_minimal.launch.py`: Unity minimal
- `src/moonmapper_description/launch/unity_full.launch.py`: Unity full (teleop + cmd_vel bro)

### Robot description

- `src/moonmapper_description/urdf/moonmapper_rover.urdf.xacro`: base URDF (rocker/bogie, sensorframes)
- `src/moonmapper_description/urdf/moonmapper_rover_gazebo.urdf.xacro`: Gazebo wrapper (ros2_control + plugins + sensors)

### Noder / scripts

- `src/moonmapper_webots/moonmapper_webots/moonmapper_driver.py`: Webots driver (`/cmd_vel`→hjul, `/odom`, TF)
- `src/moonmapper_control/moonmapper_control/teleop_keyboard.py`: enkel teleop node
- `src/moonmapper_description/launch/rocker_diff_joint.py`: kinematics-node for dependent joints i RViz
- `src/moonmapper_description/launch/unity_cmd_vel_node.py`: `/cmd_vel`→`/unity/wheel_velocities`

### Messages + plugins

- `src/moonmapper_msgs/msg/TriadSpectrum.msg`: custom message
- `src/moonmapper_gz_sensors/gz_plugins/src/TriadSpectroscopy.cc`: Gazebo sensor plugin (publiserer TriadSpectrum)
- `src/moonmapper_gz_sensors/config/triad_material_map.yaml`: material mapping og spektralkanaler

---

## 7. Hvordan starte prosjektet

### Forutsetninger

- Ubuntu 24.04 (eller kompatibel)
- ROS 2 Jazzy
- Webots + `webots_ros2_driver` (for Webots stack)
- Gazebo Sim 8 (gz-sim8) + ros_gz + gz_ros2_control (for Gazebo stack)

### Install (eksempel)

```bash
sudo apt update
sudo apt install -y ros-jazzy-desktop
sudo apt install -y ros-jazzy-webots-ros2
sudo apt install -y ros-jazzy-slam-toolbox ros-jazzy-nav2-bringup ros-jazzy-robot-localization
sudo apt install -y ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge ros-jazzy-gz-ros2-control ros-jazzy-ros2-controllers
```

> **Dette bør verifiseres**: eksakte APT-pakkenavn kan variere.

### Bygg + source

```bash
cd moonmapper_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### Kjør Webots (anbefalt først)

```bash
ros2 launch moonmapper_bringup sim.launch.py
```

Vanlige varianter:

```bash
ros2 launch moonmapper_bringup sim.launch.py use_teleop:=true
ros2 launch moonmapper_bringup sim.launch.py use_ekf:=true
ros2 launch moonmapper_bringup sim.launch.py use_ekf:=true ekf_publish_tf:=false
ros2 launch moonmapper_bringup sim.launch.py use_slam:=true use_nav2:=true
```

### Kjør Gazebo Level 1 sensors

```bash
ros2 launch moonmapper_bringup sensor_bringup.launch.py
```

### Unity

```bash
ros2 launch moonmapper_description unity_minimal.launch.py
ros2 launch moonmapper_description unity_full.launch.py
```

---

## 8. Avhengigheter

- **ROS 2**: `rclpy`, `launch(_ros)`, `robot_state_publisher`, `rviz2`, `xacro`, `joint_state_publisher(_gui)`
- **Valgfritt**: `robot_localization`, `slam_toolbox`, `nav2_*`
- **Gazebo**: `gz-sim8`, `ros_gz_sim`, `ros_gz_bridge`, `gz_ros2_control`, `diff_drive_controller`, `joint_state_broadcaster`
- **C++ deps** (plugins): `libgz-sim8-dev`, `libgz-plugin2-dev`, `yaml_cpp_vendor`
- **Webots**: `webots_ros2_driver`

---

## 9. Konfigurasjon

- `src/moonmapper_description/config/wheel_controllers.yaml`: ros2_control controllers (Gazebo)
- `src/moonmapper_description/config/ros_gz_bridge.yaml`: topic bridge mapping (Gazebo)
- `src/moonmapper_description/config/unity_params.yaml`: ros_tcp_endpoint + unity cmd_vel bridge parametre
- `src/moonmapper_localization/config/ekf.yaml`: EKF parametre
- `src/moonmapper_navigation/config/*.yaml`: Nav2 parametre
- `src/moonmapper_slam/config/slam_toolbox_params.yaml`: SLAM parametre

---

## 10. Robotmodell og kinematikk

**Rocker/bogie**:
- `rocker_left_joint`, `rocker_right_joint`: revolute (Y)
- `bogie_left_joint`, `bogie_right_joint`: revolute (Y)

**Hjul**:
- `wheel_l{1,2,3}_joint`, `wheel_r{1,2,3}_joint`: continuous

**Differential / walking-beam**:
- `rocker_bogie_diff_joint`: revolute (Z)
- RViz: mimic/kinematikk-node
- Gazebo: PD-constraint plugin (skrudd av/på med xacro-arg `enable_diff_plugin`)

Sensorframes ligger i base-URDF og brukes både av RViz og Gazebo (via `gz_frame_id`).

---

## 11. Sensorer og dataflow (Gazebo)

Se `src/moonmapper_description/urdf/moonmapper_rover_gazebo.urdf.xacro` og `src/moonmapper_description/config/ros_gz_bridge.yaml`.

- `/imu/data`
- `/stereo/left/image_raw`, `/stereo/right/image_raw` (+ camera_info)
- `/depth/*` aliases + `/depth_camera/*` original topics
- `/microscope/*`
- `/triad_sensor_{1,2}/spectrum` (`moonmapper_msgs/msg/TriadSpectrum`)

---

## 12. Status på prosjektet

**Ser ut til å fungere (implementert)**:
- Webots bringup med driver + TF/odom
- Gazebo bringup med ros2_control + sensorsuite + plugins
- Unity integrasjon på ROS-siden (endpoint + bridges)

**Bør verifiseres manuelt**:
- Unity-sceneoppsett og end-to-end topics (Unity-side)
- Fysikk-tuning (masser/inertias, friksjon, stability)

---

## 13. Kjente problemer / begrensninger

- **EKF + driver kan gi dobbel TF** (`odom->base_link`). Bruk `ekf_publish_tf:=false` om nødvendig.
- **Gazebo diff_drive_controller bruker TwistStamped** (se `wheel_controllers.yaml` og `Gazebo_test.md`).
- **Legacy scripts** for å “flytte” Gazebo/Unity assets er ikke anbefalt (se `docs/LEGACY_SCRIPTS.md`).
- `src/moonmapper_description/moonmapper_rover.urdf` er **0 bytes** (trolig artefakt/placeholder).

---

## 14. Videre arbeid

- Gjør workspace til et git-repo og legg på CI som kjører `colcon build`.
- Legg inn minimale smoke-tester for launch og topic-eksistens.
- Standardiser odom/TF-eierskap (driver vs EKF).
- Konsolider asset pipeline (root `meshes/` vs package `meshes/`) og dokumenter eksport.
- Lag en “feature matrix” for Webots vs Gazebo vs Unity.

---

## 15. Kommando-referanse

```bash
source /opt/ros/jazzy/setup.bash
cd moonmapper_ws
colcon build --symlink-install
source install/setup.bash
```

Webots:

```bash
ros2 launch moonmapper_bringup sim.launch.py
ros2 run moonmapper_control teleop_keyboard
```

Gazebo Level 1 sensors:

```bash
ros2 launch moonmapper_bringup sensor_bringup.launch.py
```

Gazebo teleop (TwistStamped):

```bash
ros2 topic pub --rate 10 /diff_drive_controller/cmd_vel geometry_msgs/msg/TwistStamped \
  '{header: {frame_id: "base_link"}, twist: {linear: {x: 0.02}, angular: {z: 0.0}}}'
```

---

## 16. Ordliste

- **URDF**: robot kinematikk/visual/collision.
- **Xacro**: macro-system som genererer URDF.
- **TF**: transform tree (frames).
- **Node / topic**: ROS-kommunikasjonsprimitiver.
- **Rocker / bogie**: terrengtilpasningsledd på rovere.
- **Diff / walking-beam**: mekanisk kobling som håndhever rocker-sum-constraint.

---

## 17. Konklusjon

MoonMapper-workspace’et kombinerer robotbeskrivelse + sim-stacks for Webots og Gazebo, pluss Unity-integrasjon. For onboarding: start med Webots bringup (`moonmapper_bringup/sim.launch.py`), og bruk Gazebo bringup når du trenger Level 1 sensorer og plugin-basert dynamikk.

---

## Manglende dokumentasjon som bør legges til

- End-to-end Unity guide som er verifisert i faktisk Unity-prosjekt.
- Canonical pipeline for mesh-generering og unity_export.
- Feature matrix (Webots vs Gazebo vs Unity) og topic/frames per stack.

