## MoonMapper – funksjonelle tester (kommandoer + forventet resultat)

Dette dokumentet er en **kombinert testplan** for workspace’et `moonmapper_ws/` basert på faktisk implementasjon i repoet:

- Webots-stack: `moonmapper_bringup/sim.launch.py` + `moonmapper_webots/moonmapper_driver.py`
- Gazebo-stack (gz-sim8): `moonmapper_bringup/sensor_bringup.launch.py` → `moonmapper_description/gazebo_rover.launch.py`
- Unity-stack: `moonmapper_description/unity_{minimal,full}.launch.py` + `ros_tcp_endpoint` + `unity_cmd_vel_node.py`

Der det er usikkert eller avhenger av ekstern setup (f.eks. Unity-scene), er det tydelig merket som **bør verifiseres manuelt**.

---

## 0) Forutsetninger (felles)

Åpne en terminal og kjør:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/rover/simulasjon/moonmapper_ws
```

Bygg:

```bash
colcon build --symlink-install
source install/setup.bash
```

**Forventet resultat**

- `colcon` ender med `Summary: ... packages finished`
- `source install/setup.bash` gir ingen feil

---

## 1) Røyk-test: pakkene finnes og kan launch’es

### 1.1 List pakker

```bash
ros2 pkg list | grep -E "moonmapper_|ros_tcp_endpoint"
```

**Forventet resultat**

- Du ser minst:
  - `moonmapper_bringup`
  - `moonmapper_description`
  - `moonmapper_webots`
  - `moonmapper_control`
  - `moonmapper_msgs`
  - `moonmapper_gz_sensors`
  - `moonmapper_navigation`, `moonmapper_slam`, `moonmapper_localization`
  - `ros_tcp_endpoint`

### 1.2 List launch-filer (sanity)

```bash
ros2 pkg prefix moonmapper_bringup
ros2 pkg prefix moonmapper_description
```

**Forventet resultat**

- Kommandoene returnerer en install-prefix path (ikke tom/feil).

---

## 2) Webots-stack: bringup + cmd_vel + odom + TF

### 2.1 Start Webots bringup (Terminal A)

```bash
ros2 launch moonmapper_bringup sim.launch.py
```

**Forventet resultat**

- Webots åpner world (GUI) og starter driveren.
- RViz åpner (default `use_rviz:=true` i launch).

> **Merk**: `sim.launch.py` starter `robot_state_publisher` med URDF fra `moonmapper_description/urdf/moonmapper_rover.urdf.xacro`.

### 2.2 Verifiser grunn-topics (Terminal B)

```bash
ros2 topic list | grep -E "^/cmd_vel$|^/odom$|^/tf$|^/tf_static$"
```

**Forventet resultat**

- Minst `/cmd_vel`, `/odom`, `/tf` (og ofte `/tf_static`).

### 2.3 Echo odometri (Terminal B)

```bash
ros2 topic echo /odom --once
```

**Forventet resultat**

- En `nav_msgs/Odometry` melding med:
  - `header.frame_id: "odom"`
  - `child_frame_id: "base_link"`

### 2.4 Publiser en enkel cmd_vel (Terminal B)

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.05}, angular: {z: 0.0}}"
```

**Forventet resultat**

- Roveren beveger seg fremover i Webots.
- `/odom` endrer posisjon over tid.

Stopp:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

### 2.5 TF sanity (Terminal B)

```bash
ros2 run tf2_tools view_frames
```

**Forventet resultat**

- PDF/Graph genereres i current directory.
- TF-tree inkluderer minst `odom -> base_link` (publisert av Webots-driveren).

### 2.6 Teleop node (valgfritt)

Start bringup med teleop:

```bash
ros2 launch moonmapper_bringup sim.launch.py use_teleop:=true
```

**Forventet resultat**

- Node `teleop_keyboard` starter og publiserer `/cmd_vel` ved tastetrykk.

---

## 3) Webots + EKF (robot_localization) – konflikt-sjekk

> Bakgrunn: Webots-driver publiserer TF `odom->base_link`. EKF kan også publisere TF.

### 3.1 Start med EKF (Terminal A)

```bash
ros2 launch moonmapper_bringup sim.launch.py use_ekf:=true
```

**Forventet resultat**

- EKF node starter uten crash.

### 3.2 Hvis du får TF-konflikt (bør verifiseres manuelt)

Se etter symptomer i RViz (TF “jumping”) eller warnings om multiple publishers.

Mitigasjon (kjør EKF uten TF):

```bash
ros2 launch moonmapper_bringup sim.launch.py use_ekf:=true ekf_publish_tf:=false
```

**Forventet resultat**

- EKF kjører, men publiserer ikke `odom->base_link` TF (unngår dobbel TF).

---

## 4) Gazebo (gz-sim8): bringup + controllers + odom + TF + sensors

### 4.1 Start Gazebo Level 1 bringup (Terminal A)

```bash
ros2 launch moonmapper_bringup sensor_bringup.launch.py
```

**Forventet resultat**

- Gazebo GUI åpnes med world (default `moon_arena.sdf`).
- Roveren spawner (modellnavn `moonmapper` i launch).
- `joint_state_broadcaster` og `diff_drive_controller` blir aktivert (via spawners).

> Tips: Roveren er veldig liten (~5 cm). Zoom inn og bruk Entity Tree til å fokusere `moonmapper`.

### 4.2 Verifiser controllers (Terminal B)

Hvis `ros2 control` finnes:

```bash
ros2 control list_controllers
```

**Forventet resultat**

- `joint_state_broadcaster` er `active`
- `diff_drive_controller` er `active`

Hvis `ros2 control` ikke er installert:

```bash
ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers
```

### 4.3 Verifiser topics (Terminal B)

```bash
ros2 topic list | grep -E "joint_states|diff_drive_controller|/clock|/tf$|imu|stereo|depth|microscope|triad_sensor"
```

**Forventet resultat**

- Minst:
  - `/clock`
  - `/tf`
  - `/joint_states`
  - `/diff_drive_controller/cmd_vel`
  - `/diff_drive_controller/odom`
- Sensor topics (se 4.5–4.7)

### 4.4 Kjør rover med TwistStamped (Terminal B)

Repoets `wheel_controllers.yaml` har `use_stamped_vel: true`, så test med `TwistStamped`:

```bash
ros2 topic pub --times 30 -r 10 /diff_drive_controller/cmd_vel \
  geometry_msgs/msg/TwistStamped \
  '{header: {frame_id: "base_link"}, twist: {linear: {x: 0.02}, angular: {z: 0.0}}}'
```

**Forventet resultat**

- Roveren beveger seg fremover i Gazebo.
- `/diff_drive_controller/odom` endrer seg:

```bash
ros2 topic echo /diff_drive_controller/odom --field pose.pose.position
```

### 4.5 IMU (Terminal B)

```bash
ros2 topic hz /imu/data
ros2 topic echo /imu/data --once
```

**Forventet resultat**

- `hz` viser ca. 100 Hz (eller i nærheten).
- `/imu/data` inneholder gyldige tall, og `header.frame_id` er `imu_link`.

**Tolkning (hurtig)**

- `linear_acceleration.z` inkluderer tyngdeakselerasjon (skal typisk ligge rundt g når roveren står stille, med fortegn avhengig av orientering).
- `angular_velocity` bør være nær 0 når roveren står stille.
- `orientation` er en quaternion; det viktige for “plassering” er `frame_id` + at TF mellom `base_link` og `imu_link` er korrekt (se 4.10).

**Eksempel-output (fra test hos dere) + hva det betyr**

```text
header.frame_id: imu_link
angular_velocity:
  x: 0.0005766
  y: 0.0156736
  z: 0.1019686
linear_acceleration:
  x: -0.0026075
  y: -0.0796006
  z: 1.1149288
orientation (quat):
  x: -0.0035024
  y: -0.0206467
  z: -0.1286889
  w: 0.9914639
```

- `**frame_id: imu_link**`: bra – det betyr at data er stemplet i IMU-rammen du har i URDF.
- `**angular_velocity**`: enheten er rad/s. Når roveren står stille forventer vi “nær 0”, men litt støy/drift er normalt. Her er `z≈0.10 rad/s` (\approx 5.7^\circ/s) litt høyt hvis roveren faktisk står helt stille; ofte skyldes det at modellen fortsatt har litt mikrobevegelse i sim (kontakt/joint-svingninger) eller at du tok prøven mens den beveget seg.
- `**linear_acceleration**`: enheten er m/s². I en ideell IMU i rolig hvile vil størrelsen være ca. g≈9.81. I Gazebo kan “gravitasjon i IMU” og støy/skalering variere med sensormodellen. At du ser `z≈1.11` betyr: enten (a) IMU-modellen rapporterer “uten full g” (vanlig i noen sim-oppsett), eller (b) IMU-aksene er ikke alignet slik du intuitivt forventer. Derfor er beste sjekk å se på **magnituden** og sammenligne mot orientering:
  - Kjør `ros2 topic echo /imu/data --once` flere ganger mens roveren står helt stille og se om aksene holder seg stabile.
  - Hvis du vil verifisere “g”, kan du midlertidig tippe roveren i Gazebo og se om aksen som peker opp/ned endrer seg.
- `**orientation`**: quaternionen er nær identitet (`w≈0.99`), som typisk betyr liten rotasjon fra referansen ved måletidspunktet. Dette er normalt.

### 4.6 Stereo (Terminal B)

```bash
ros2 topic hz /stereo/left/image_raw
ros2 topic hz /stereo/right/image_raw
ros2 topic echo /stereo/left/camera_info --once
```

**Forventet resultat**

- Frekvens ca. 15 Hz (som i xacro).
- `camera_info` finnes.

Valgfri visning:

```bash
QWindow
ros2 run rqt_image_view rqt_image_view
QT
QT_QPA_PLATFORM=xcb ros2 run image_tools showimage --ros-args -r image:=/stereo/left/image_raw
QT_QPA_PLATFORM=xcb ros2 run image_tools showimage --ros-args -r image:=/stereo/right/image_raw
```

### 4.7 RGB-D / Depth (Terminal B)

Alias scheme (Level 1):

```bash
ros2 topic hz /depth/image_raw
ros2 topic hz /depth/image_depth
ros2 topic hz /depth/points
ros2 topic echo /depth/camera_info --once
```

Original topics (kompatibilitet):

```bash
ros2 topic hz /depth_camera/image
ros2 topic hz /depth_camera/depth_image
ros2 topic hz /depth_camera/points
```

**Forventet resultat**

- Topics finnes og publiserer data (ca. 15 Hz).

### 4.8 Microscope (Terminal B)

```bash
ros2 topic hz /microscope/image_raw
ros2 topic echo /microscope/camera_info --once
```

**Forventet resultat**

- Topic eksisterer og publiserer (ca. 10 Hz).
- `frame_id` bør være `mikroskop_optical_frame` (via `gz_frame_id` i xacro).

### 4.9 Triad spectroscopy (synthetic) (Terminal B)

```bash
ros2 topic hz /triad_sensor_1/spectrum
ros2 topic echo /triad_sensor_1/spectrum --once
ros2 topic echo /triad_sensor_2/spectrum --once
```

**Forventet resultat**

- `moonmapper_msgs/msg/TriadSpectrum` meldinger kommer (default 20 Hz).
- `material_label` endrer seg *potensielt* ved å kjøre over ulike objekter (avhenger av world og `triad_material_map.yaml`).

> **Bør verifiseres manuelt**: at raycast faktisk treffer entity names i din world (plugin har fallback zone rules i YAML).

### 4.10 Plasseringstest (TF) for IMU, halleffekt og triad (Terminal B)

Dette tester “hvor sensoren sitter” i roboten via TF (uavhengig av topic-data).

#### 4.10.1 IMU plassering

```bash
ros2 run tf2_ros tf2_echo base_link imu_link
```

**Forventet resultat**

- `Translation:` matcher URDF (`imu_joint` i `moonmapper_rover.urdf.xacro`), typisk små tall (mm–cm).
- `Rotation:` kan være identitet hvis `rpy="0 0 0"` i URDF.

Feiltypisk:

- Store/rare tall → feil frame, feil URDF, eller du har glemt å respawne i Gazebo etter endring.

#### 4.10.2 Halleffekt-sensor plassering

Halleffekt er implementert som en link/joint i URDF (TF), ikke som en egen Gazebo “sensor topic”.

```bash
ros2 run tf2_ros tf2_echo rocker_right_link halleffekt_sensor_link
ros2 run tf2_ros tf2_echo base_link halleffekt_sensor_link
```

**Forventet resultat**

- `rocker_right_link -> halleffekt_sensor_link` matcher joint-origo i URDF.
- `base_link -> halleffekt_sensor_link` endrer seg når rockeren beveger seg (fordi den sitter på rockeren).

#### 4.10.3 Triad-sensor plassering (TF-linkene)

```bash
ros2 run tf2_ros tf2_echo base_link triad_sensor_1_link
ros2 run tf2_ros tf2_echo base_link triad_sensor_2_link
```

**Forventet resultat**

- `Translation:` matcher `triad_sensor_1_joint` / `triad_sensor_2_joint` i URDF.

Viktig: dette tester kun TF-plasseringen av linkene. Selve “målepunktet” i Triad-pluginen er raycast-starten. Se 4.11.

### 4.11 Triad “målepunkt” (raycast) sanity (Terminal B)

Dette verifiserer at Triad-pluginen faktisk publiserer, og at meldingene er stemplet i forventet frame.

```bash
ros2 topic hz /triad_sensor_1/spectrum
ros2 topic echo /triad_sensor_1/spectrum --once
ros2 topic echo /triad_sensor_2/spectrum --once
```

**Forventet resultat**

- `hz` rundt 20 Hz (default i `moonmapper_rover_gazebo.urdf.xacro`).
- `header.frame_id` bør være `triad_sensor_1_link` / `triad_sensor_2_link` (eller det som er konfigurert i pluginen).
- `material_label` kan være konstant hvis du står stille over samme “sone/materiale” i world/YAML.

Feiltypisk:

- Topic finnes ikke → plugin ikke lastet / feil `GZ_SIM_SYSTEM_PLUGIN_PATH` / pakken ikke bygget.
- Data finnes men “feil sted” → TF er riktig, men raycast-offset i pluginen matcher ikke URDF-jointene (må justeres i `moonmapper_gz_sensors`).

---

## 5) RViz-only: vis robotbeskrivelse og kinematisk coupling

### 5.1 Vis robot i RViz (uten sim)

```bash
ros2 launch moonmapper_description view_robot.launch.py
```

**Forventet resultat**

- RViz åpner med robotmodellen.
- `joint_state_publisher_gui` lar deg justere joints.

### 5.2 Vis robot med rocker-bogie kinematics-node

```bash
ros2 launch moonmapper_description view_robot_with_diff.launch.py
```

**Forventet resultat**

- RViz åpner.
- `/joint_states_raw` kommer fra JSP GUI og `/joint_states` publiseres av `rocker_diff_joint.py`.
- Bogie/diff/hinge joints oppfører seg “koblet” når du endrer rocker-joints.

---

## 6) Unity-stack (ROS-TCP) – røyk-test fra ROS-siden

### 6.1 Start minimal Unity-stack (Terminal A)

```bash
ros2 launch moonmapper_description unity_minimal.launch.py
```

**Forventet resultat**

- `ros_tcp_endpoint` starter og lytter på port (default 10000, se `unity_params.yaml`).
- RViz starter (default `use_rviz:=true`).

> **Bør verifiseres manuelt**: at Unity faktisk kan koble til (IP/port) og publiserer `/joint_states` osv.

### 6.2 Start full Unity-stack (Terminal A)

```bash
ros2 launch moonmapper_description unity_full.launch.py
```

**Forventet resultat**

- I tillegg til minimal stack starter `unity_cmd_vel_bridge` som publiserer `/unity/wheel_velocities`.
- Hvis `xterm` er installert og `start_teleop:=true`, starter teleop_twist_keyboard i eget vindu.

### 6.3 Verifiser wheel velocity topic (Terminal B)

```bash
ros2 topic echo /unity/wheel_velocities --once
```

**Forventet resultat**

- `std_msgs/Float64MultiArray` med 6 elementer:
  - `[wheel_l1, wheel_l2, wheel_l3, wheel_r1, wheel_r2, wheel_r3]` i rad/s

Send cmd_vel:

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.02}, angular: {z: 0.0}}"
```

**Forventet resultat**

- `/unity/wheel_velocities` endrer seg (ikke null).

---

## 7) “Dårlige dager”: rask feilsøking

### 7.1 Byggfeil / rare symlink-feil

Hvis du får “failed to create symbolic link … existing path cannot be removed”, rydd stale artifacts:

```bash
rm -rf build/<pakkenavn> install/<pakkenavn>
colcon build --symlink-install
```

### 7.2 Ingen data på topics

Sjekk at:

```bash
ros2 topic list
ros2 topic hz /clock
```

**Forventet resultat**

- I Gazebo sim skal `/clock` tikke.
- I Webots kan `use_sim_time` være relevant (default true i bringup).

---

## Referanser i repoet

- Gazebo testing: `src/moonmapper_description/Gazebo_test.md`
- Level 1 sensors checklist: `src/moonmapper_description/Level1_Sensors.md`
- Konfig:
  - `src/moonmapper_description/config/ros_gz_bridge.yaml`
  - `src/moonmapper_description/config/wheel_controllers.yaml`
  - `src/moonmapper_description/config/unity_params.yaml`
  - `src/moonmapper_gz_sensors/config/triad_material_map.yaml`

