## MoonMapper rover – fysikk- og metadataoversikt

Dette dokumentet samler **pose/geometri**, **kinematikk**, **dynamikk** og **simulator-spesifikke fysikkparametere** for MoonMapper-roveren, basert på det som faktisk er implementert i:

- `src/moonmapper_description/urdf/moonmapper_rover.urdf.xacro` (base URDF: kinematikk + inertial + sensorframes + transmissions)
- `src/moonmapper_description/urdf/moonmapper_rover_gazebo.urdf.xacro` (Gazebo wrapper: kontakt/friksjon, ros2_control, plugins, sensors)
- `src/moonmapper_description/config/wheel_controllers.yaml` (diff_drive_controller parametre i Gazebo)
- `src/moonmapper_webots/moonmapper_webots/moonmapper_driver.py` (Webots: skid-steer kinematikk + odom/TF)

Hvis du endrer modell/geometri: oppdater helst **Xacro** (ikke generert URDF), og bruk dette dokumentet som sjekkliste for konsistens.

---

## 1) Koordinatsystemer og frames

### 1.1 Basisframes (URDF)

Implementert i base URDF:
- `base_footprint` (egen link)
- `base_link` (chassis)
- fast joint: `base_footprint_to_base_link` med origin `xyz="0 0 0" rpy="0 0 0"`

**Implementert i**
- `src/moonmapper_description/urdf/moonmapper_rover.urdf.xacro`

### 1.2 Odom frame (sim)

- **Webots** publiserer `odom -> base_link` TF i driveren.
- **Gazebo** diff_drive_controller bruker `odom_frame_id: odom` og `base_frame_id: base_footprint`.

**Implementert i**
- Webots TF: `src/moonmapper_webots/moonmapper_webots/moonmapper_driver.py`
- Gazebo controller frames: `src/moonmapper_description/config/wheel_controllers.yaml`

> **Merk**: Dette gir en blanding av `base_link` og `base_footprint` som child/base frame mellom stacks. Det kan være OK, men bør standardiseres hvis du vil ha identisk TF mellom simulatorer.

---

## 2) Geometri og poses (kinematisk “source of truth”)

I denne modellen er **poses definert som joint origins**. Mesh-geometrien refereres via `package://moonmapper_description/meshes/...`.

### 2.1 Chassis (base_link)

- Visual/collision mesh: `meshes/visual/robot_base.stl`
- Inertial:
  - masse: `2.0` kg
  - inertiamatrise:
    - `ixx=0.0010`, `iyy=0.0012`, `izz=0.0015`
  - inertial origin: `xyz="0 0 -0.005"`

**Implementert i**
- `src/moonmapper_description/urdf/moonmapper_rover.urdf.xacro` (`link name="base_link"`)

### 2.2 Rocker/Bogie (høyre side – eksempel)

**Rocker pivot**
- `rocker_right_joint`: revolute om Y
- origin: `xyz="0.014227 -0.065999 0.021756"`
- limits: ±30° (konvertert via `deg`-property), `velocity=2.0`, `effort=100`
- joint dynamics: `damping=0.02`, `friction=0.005`

**Bogie pivot**
- `bogie_right_joint`: revolute om Y
- origin: `xyz="0.087545 -0.025322 -0.050495"`
- limits: ±40°, `velocity=2.0`, `effort=100`
- joint dynamics: `damping=0.01`, `friction=0.003`

**Implementert i**
- `src/moonmapper_description/urdf/moonmapper_rover.urdf.xacro` (joints `rocker_right_joint`, `bogie_right_joint`)

### 2.3 Hjul (høyre side – eksempel)

- `wheel_r1_joint`: continuous om Y
  - origin: `xyz="-0.157986 -0.023379 -0.091195"`
  - `damping=0.005`, `friction=0.002`
  - `effort=0.5`, `velocity=30.0`

Alle hjul har inertial `mass=0.02` kg og små inertier (se URDF).

**Implementert i**
- `src/moonmapper_description/urdf/moonmapper_rover.urdf.xacro` (wheel links/joints)

### 2.4 “Walking-beam / differential”-ledd

Base URDF definerer:
- `rocker_bogie_diff_joint`: revolute om Z
  - parent: `feste_rocker_karosseri_link`
  - origin: `xyz="0.000012 0.000001 0.014"`
  - dynamics: `damping=0.8`, `friction=0.08`
  - limits: ±90°
  - mimic (for RViz-konsistens): `multiplier="0.82"` mot `rocker_right_joint`

**Implementert i**
- `src/moonmapper_description/urdf/moonmapper_rover.urdf.xacro` (joint `rocker_bogie_diff_joint`)

> I Gazebo brukes en plugin som håndhever “sum-constraint” på rocker-leddene og driver diff-leddet via aux PD (se seksjon 5).

---

## 3) Kinematikk (grader av frihet og drivlinje)

### 3.1 DOF-oversikt (URDF)

URDF beskriver et tre (ingen lukkede løkker):

- Rockers (passive i sim, revolute Y):
  - `rocker_left_joint`, `rocker_right_joint`
- Bogies (passive i sim, revolute Y):
  - `bogie_left_joint`, `bogie_right_joint`
- Hjul (aktive, continuous Y):
  - `wheel_l{1,2,3}_joint`, `wheel_r{1,2,3}_joint`
- Diff-stav (revolute Z):
  - `rocker_bogie_diff_joint`

**Implementert i**
- `src/moonmapper_description/urdf/moonmapper_rover.urdf.xacro` (kommentar “DOF-oversikt” + joints)

### 3.2 Webots “skid-steer” kinematikk

Webots-driveren implementerer enkel differential/skid-steer:

- konstanter:
  - `WHEEL_RADIUS = 0.08` m
  - `HALF_TRACK = 0.175` m
- mapping:
  - `left_vel = (v - ω * HALF_TRACK) / WHEEL_RADIUS`
  - `right_vel = (v + ω * HALF_TRACK) / WHEEL_RADIUS`
- setter samme `left_vel` på tre venstre motorer og `right_vel` på tre høyre.

**Implementert i**
- `src/moonmapper_webots/moonmapper_webots/moonmapper_driver.py`

> **Viktig**: Dette er en annen geometri enn Gazebo-konfigen (`wheel_radius=0.0073`, `wheel_separation=0.030`). Det kan være bevisst (to ulike skalaer) eller en inkonsistens. **Dette bør verifiseres** mot Webots robotmodell og meshes.

---

## 4) Dynamikk (masser, inertias, joint damping/friction)

### 4.1 Masser/inertias (utdrag)

Base URDF definerer inertial på de dynamiske linkene. Eksempler:

- `base_link`:
  - mass `2.0`
  - inertia ~ `1e-3` kgm² skala
- `rocker_*_link`:
  - mass `0.08`
  - inertia: `iyy/izz ≈ 4e-4`
- `bogie_*_link`:
  - mass `0.05`
  - inertia: `iyy/izz ≈ 1.5e-4`
- `wheel_*_link`:
  - mass `0.02`
  - inertia: `~1e-6`
- flere små bracket/rod/motor links har mass `0.001–0.010` og inertier `1e-8…`

**Implementert i**
- `src/moonmapper_description/urdf/moonmapper_rover.urdf.xacro`

### 4.2 Joint damping/friction

URDF setter `dynamics damping/friction` på:

- `rocker_*_joint`: `damping=0.02`, `friction=0.005`
- `bogie_*_joint`: `damping=0.01`, `friction=0.003`
- `wheel_*_joint`: `damping=0.005`, `friction=0.002`
- `rocker_bogie_diff_joint`: `damping=0.8`, `friction=0.08`

**Implementert i**
- `src/moonmapper_description/urdf/moonmapper_rover.urdf.xacro`

---

## 5) Gazebo-spesifikk fysikk (kontakt, ros2_control, plugins)

### 5.1 Kontakt/friksjon på hjul (Gazebo tags)

Gazebo wrapper setter hjulkontakt-parametere per hjul-link:

- `mu1 = 1.2`, `mu2 = 1.0`
- kontaktfjær: `kp = 300000.0`, `kd = 50.0`
- `minDepth=0.0005`, `maxVel=0.5`, `fdir1=1 0 0`

**Implementert i**
- `src/moonmapper_description/urdf/moonmapper_rover_gazebo.urdf.xacro`
  - properties `wheel_mu1`, `wheel_mu2`, `wheel_kp`, `wheel_kd`
  - macro `wheel_gazebo` som brukes for `wheel_{l,r}{1,2,3}_link`

### 5.2 “implicitSpringDamper” for passive joints

Wrapper aktiverer:
- `implicitSpringDamper=true` for rocker/bogie/diff joints
- `provideFeedback=true` for rockers og diff

**Implementert i**
- `src/moonmapper_description/urdf/moonmapper_rover_gazebo.urdf.xacro`

### 5.3 ros2_control (Gazebo)

Gazebo wrapper definerer `ros2_control` system:

- hardware plugin: `gz_ros2_control/GazeboSimSystem`
- wheel joints eksponeres med `velocity` command interface og pos/vel/eff state
- passive joints eksponeres state-only (pos/vel)

**Implementert i**
- `src/moonmapper_description/urdf/moonmapper_rover_gazebo.urdf.xacro` (`<ros2_control ...>`)

Controller-konfig ligger i:
- `src/moonmapper_description/config/wheel_controllers.yaml`

Viktige controller-parametre:
- `left_wheel_names`/`right_wheel_names` (3 per side)
- `wheel_separation: 0.030` m
- `wheel_radius: 0.0073` m
- `use_stamped_vel: true` (TwistStamped på `/diff_drive_controller/cmd_vel`)
- `odom_frame_id: odom`, `base_frame_id: base_footprint`

### 5.4 RockerBogieDifferential plugin (Gazebo)

Gazebo wrapper laster plugin:

- `filename="libmoonmapper_rocker_bogie_differential.so"`
- `name="moonmapper::RockerBogieDifferential"`
- parametre (utdrag):
  - rocker joints: `rocker_left_joint`, `rocker_right_joint`
  - diff joint: `rocker_bogie_diff_joint`
  - sum-PD: `kp=2.0`, `kd=0.2`, `target_sum=0.0`, `max_torque=0.2`
  - diff target: `k_diff_L=0.0`, `k_diff_R=0.82`, `k_diff_0=0.0`
  - aux PD: `kp_aux=0.2`, `kd_aux=0.02`, `max_torque_aux=0.02`
- kan skrus av/på via xacro-arg `enable_diff_plugin` (default true i bringup)

**Implementert i**
- plugin load/config: `src/moonmapper_description/urdf/moonmapper_rover_gazebo.urdf.xacro`
- C++ kilde: `src/moonmapper_description/gz_plugins/src/RockerBogieDifferential.cc`
- launch-arg for enable: `src/moonmapper_bringup/launch/sensor_bringup.launch.py` → `src/moonmapper_description/launch/gazebo_rover.launch.py`

### 5.5 Triad spectroscopy plugin (Gazebo)

Gazebo wrapper laster:
- `libmoonmapper_triad_spectroscopy.so` (`moonmapper::TriadSpectroscopy`)
- publiserer `/triad_sensor_1/spectrum` og `/triad_sensor_2/spectrum`
- bruker `moonmapper_gz_sensors/config/triad_material_map.yaml`

**Implementert i**
- plugin load/config: `src/moonmapper_description/urdf/moonmapper_rover_gazebo.urdf.xacro`
- C++ kilde: `src/moonmapper_gz_sensors/gz_plugins/src/TriadSpectroscopy.cc`
- YAML: `src/moonmapper_gz_sensors/config/triad_material_map.yaml`
- msg-type: `src/moonmapper_msgs/msg/TriadSpectrum.msg`

---

## 6) Sensorposer og “metadata” (Gazebo wrapper)

Gazebo wrapper fester sensorer direkte på `base_link` med eksplisitte poses, men bruker `gz_frame_id` til URDF-frames.

### 6.1 IMU

- pose (base_link): `0 0 0.005 0 0 0`
- update_rate: `100`
- topic: `imu` (bro’es til `/imu/data` via ros_gz_bridge)
- `gz_frame_id`: `imu_link`
- noise:
  - angular vel stddev `0.002`
  - linear accel stddev `0.017`

**Implementert i**
- `src/moonmapper_description/urdf/moonmapper_rover_gazebo.urdf.xacro` (`sensor type="imu"`)

URDF TF-link:
- `imu_joint` origin: `xyz="0.001234 -0.000060 0.003194"`

**Implementert i**
- `src/moonmapper_description/urdf/moonmapper_rover.urdf.xacro`

> **Merk**: Gazebo-IMU pose (0,0,0.005) er ikke identisk med URDF imu_joint (0.001234, -0.000060, 0.003194). Wrapper-kommentaren sier at sensorposes “må matche URDF-TF-kjeden”. **Dette bør verifiseres** (enten endre URDF joint eller Gazebo pose).

### 6.2 Stereo (venstre/høyre)

- venstre camera pose (base_link):
  - `-0.166163 -0.003930 0.069183 0 0 pi`
  - topic: `stereo/left/image_raw`
  - `gz_frame_id`: `stereo_left_optical_frame`
- høyre camera pose (base_link):
  - `-0.166163 -0.013930 0.069183 0 0 pi`
  - topic: `stereo/right/image_raw`
  - `gz_frame_id`: `stereo_right_optical_frame`
- update_rate: `15`
- HFOV: `1.2217305` (70°)
- image: `320x240`, format `R8G8B8`

**Implementert i**
- `src/moonmapper_description/urdf/moonmapper_rover_gazebo.urdf.xacro`

URDF TF-kjede:
- `stereo_camera_joint` origin: `-0.166163 -0.008930 0.069183`
- venstre/høyre offset på ±0.005 i Y
- optical frame rpy: `-pi/2 0 -pi/2`

**Implementert i**
- `src/moonmapper_description/urdf/moonmapper_rover.urdf.xacro`

### 6.3 RGB-D

- pose (base_link): `-0.166163 -0.008930 0.069183 0 0 pi`
- topic base: `depth_camera` (gir `/depth_camera/*`)
- update_rate: `15`
- near/far: `0.05/10.0` (merknad: clampes til 0.1 av renderer)
- `gz_frame_id`: `depth_camera_optical_frame`

**Implementert i**
- `src/moonmapper_description/urdf/moonmapper_rover_gazebo.urdf.xacro`

### 6.4 Mikroskop

- pose (base_link): `-0.091849 -0.001046 -0.006375 -pi/2 0 -pi/2`
- update_rate: `10`
- topic: `microscope/image_raw`
- `gz_frame_id`: `mikroskop_optical_frame`
- HFOV: `60°`, `320x240`

**Implementert i**
- `src/moonmapper_description/urdf/moonmapper_rover_gazebo.urdf.xacro`

URDF TF-link:
- `mikroskop_joint` origin: `-0.091849 -0.001046 -0.006375`
- optical frame rpy: `-pi/2 0 -pi/2`

**Implementert i**
- `src/moonmapper_description/urdf/moonmapper_rover.urdf.xacro`

---

## 7) “Hvor er dette implementert?” – rask indeks

### Kinematikk (URDF joints/origins)
- `src/moonmapper_description/urdf/moonmapper_rover.urdf.xacro`

### Gazebo fysikk (kontakt + ros2_control + plugins + sensors)
- `src/moonmapper_description/urdf/moonmapper_rover_gazebo.urdf.xacro`

### Gazebo controllers (hjulgeometri, cmd_vel type, frames)
- `src/moonmapper_description/config/wheel_controllers.yaml`

### Webots kinematikk og odometri
- `src/moonmapper_webots/moonmapper_webots/moonmapper_driver.py`

### Gazebo Level 1 “Triad spectroscopy” detaljer
- plugin: `src/moonmapper_gz_sensors/gz_plugins/src/TriadSpectroscopy.cc`
- mapping: `src/moonmapper_gz_sensors/config/triad_material_map.yaml`
- msg: `src/moonmapper_msgs/msg/TriadSpectrum.msg`

---

## 8) Konsistens-sjekkliste (anbefalt)

Hvis du endrer skala/mesh/geometri, verifiser:

1. **wheel_radius / wheel_separation** er konsistente mellom:
   - `wheel_controllers.yaml` (Gazebo)
   - `unity_params.yaml` (Unity bridge; hvis i bruk)
   - Webots driver (`WHEEL_RADIUS`, `HALF_TRACK`) og Webots robotmodell
2. Sensor **pose** i Gazebo wrapper matcher URDF TF (eller du dokumenterer bevisst avvik).
3. `base_frame_id`/`child_frame_id` i odometri/TF er konsistente mellom simulatorene.

