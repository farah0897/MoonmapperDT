# MoonMapper – Gazebo Sim 8 testguide

Denne README-en forklarer hvordan du starter simuleringen og verifiserer at rocker-bogie-suspensjonen, hjuldriften og den virtuelle differensialen
oppfører seg som forventet.

Pakken kombinerer:

- URDF/Xacro-beskrivelse av roveren (`urdf/moonmapper_rover.urdf.xacro`)
- Gazebo-wrapper med fysikk-tagger, `ros2_control` og plugin-lasting
  (`urdf/moonmapper_rover_gazebo.urdf.xacro`)
- En egenskrevet Gazebo Sim 8 system-plugin som håndhever det virtuelle
  differensial-leddet via PD-moment (`gz_plugins/src/RockerBogieDifferential.cc`)
- `diff_drive_controller` for 6-hjulsdrift
- En testverden med bakke, rampe, trinn og steiner
  (`worlds/rocker_bogie_test.sdf`)

---

## 1. Forutsetninger

- **ROS 2 Jazzy**
- **Gazebo Sim 8** (Harmonic) – kommer med `ros-jazzy-ros-gz`
- `ros-jazzy-gz-ros2-control`, `ros-jazzy-ros2-controllers`, `ros-jazzy-xacro`

Bygg én gang:

```bash
cd ~/rover/simulasjon/moonmapper_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select moonmapper_description --symlink-install
source install/setup.bash
```

Etter hver kjerne-endring (xacro, plugin, YAML) må du `source install/setup.bash`
på nytt i ALLE åpne terminaler.

---

## 2. Terminaloppsett

Bruk minst **3 terminaler**. I hver terminal, først:

```bash
source /opt/ros/jazzy/setup.bash
source ~/rover/simulasjon/moonmapper_ws/install/setup.bash
```

| Terminal | Rolle |
|---|---|
| **1** | Starter Gazebo + RViz + controllers (kjører kontinuerlig) |
| **2** | Sender `cmd_vel`-kommandoer (kjøring og sving) |
| **3** | Observerer `/joint_states`, `/odom`, `/tf` |

---

## 3. Start simuleringen

I **Terminal 1** (standard test-verden):

```bash
ros2 launch moonmapper_description gazebo_rover.launch.py
```

**Alternativ: oppdragsverden med sand-heightmap + inngjerdet arena**

```bash
ros2 launch moonmapper_description gazebo_rover.launch.py \
    world:=$(ros2 pkg prefix moonmapper_description)/share/moonmapper_description/worlds/moon_arena.sdf \
    spawn_z:=0.5
```

Innhold i `moon_arena.sdf`:

- **Terreng**: sand-mesh fra Fuel-modellen `hmoyen/occluded sand
  heightmap` (native 1.02 × 1.10 × 0.21 m, skalert til **4.1 × 4.4 ×
  0.31 m** og sentrert i origo). Lastes ned foerste oppstart og
  cachet i `~/.gz/fuel/`.
- **Arena**: 4 gule vegger som enclosser et **4.5 × 4.5 m (≈ 20 m²)**
  rektangel sentrert i origo. Veggene er 1 m hoeye og gaar 0.25 m ned
  i bakken.
- **Fallback-bakkeplan**: 20 × 20 m flatt plan paa z=0 som fanger
  roveren hvis den havner paa kanten av sand-patchet.
- **Lyssetting**: lavt maane-likt sollys fra oest med svakt fyllys,
  for tydelige skygger i stereo- og depth-kamera.

> **Finn roveren i Gazebo-GUI**: den er bare ~5 cm lang, så du må
> zoome kraftig inn paa origo (rund midten av arenaen) for aa se
> den. I `Entity Tree`-panelet: dobbel-klikk `moonmapper` for aa
> flytte kameraet til den. I RViz, sett `Fixed Frame = base_link` for
> aa sentrere synet paa roveren.

**Forventet resultat:**

- Gazebo-GUI åpnes med verden `rocker_bogie_test` (ground plane, rampe,
  trinn, tre små steiner) ELLER `moon_arena` (heightmap + 4 vegger).
- Roveren `moonmapper` spawner over bakken og faller ned på hjulene.
- I log-en skal du se:

  ```
  [gazebo-1] [Msg] [RockerBogieDifferential] Configured for model 'moonmapper':
  [gazebo-1]   left_rocker_joint  = rocker_left_joint
  [gazebo-1]   right_rocker_joint = rocker_right_joint
  [gazebo-1]   kp=2  kd=0.2  target_sum=0  max_torque=0.5
  [gazebo-1] [INFO] [spawner_joint_state_broadcaster]: Configured and activated joint_state_broadcaster
  [gazebo-1] [INFO] [spawner_diff_drive_controller]: Configured and activated diff_drive_controller
  ```

- RViz-vindu viser roveren med alle links i status `Transform OK` (grønn hake).

Hvis roveren eksploderer / synker gjennom bakken, se seksjon 8 (feilsøking).

---

## 4. Test A – Sanity-sjekk at alt lever

I **Terminal 2**:

```bash
# Begge kontrollere skal staa som [active]
# (krever pakken ros-jazzy-ros2controlcli)
ros2 control list_controllers

# Alternativ uten ros2controlcli:
ros2 service call /controller_manager/list_controllers \
  controller_manager_msgs/srv/ListControllers

# Forventet topics
ros2 topic list | grep -E "joint_states|cmd_vel|odom"
```

Hvis `ros2 control` sier `invalid choice: 'control'`, installer CLI-pakken:

```bash
sudo apt install -y ros-jazzy-ros2controlcli
```

**Forventet resultat:**

```
diff_drive_controller        diff_drive_controller/DiffDriveController  active
joint_state_broadcaster      joint_state_broadcaster/JointStateBroadcaster  active
```

Topics skal inneholde minst:

```
/joint_states
/diff_drive_controller/cmd_vel         # TwistStamped (Jazzy v4.39+)
/diff_drive_controller/odom
/tf
/clock
```

> **OBS – Jazzy-endring:** `diff_drive_controller` i ROS 2 Jazzy abonnerer kun på
> `geometry_msgs/msg/TwistStamped` på `/diff_drive_controller/cmd_vel`. Den gamle
> `/diff_drive_controller/cmd_vel_unstamped` med `Twist` finnes **ikke lenger**.
> Bruk derfor `TwistStamped` i alle `ros2 topic pub`-kommandoer under.

---

## 5. Test B – Kjøring fram og bak

### 5.1 Kjør rett fram

**Terminal 2:**

```bash
# 3 sekunder framover (2 cm/s) og stopp deretter automatisk
ros2 topic pub --times 30 -r 10 /diff_drive_controller/cmd_vel \
  geometry_msgs/msg/TwistStamped \
  '{header: {frame_id: "base_link"}, twist: {linear: {x: 0.02}, angular: {z: 0.0}}}'
```

**Terminal 3 – observer odometri:**

```bash
ros2 topic echo /diff_drive_controller/odom --field pose.pose.position
```

**Forventet resultat:**

- I Gazebo beveger roveren seg synlig framover (~6 cm totalt).
- `pose.pose.position.x` vokser fra 0 til ca. 0.06.
- Alle seks hjul spinner i samme retning (synlig i Gazebo hvis du skrur på
  `View → Joints`).

### 5.2 Kjør bakover

```bash
# Terminal 2
ros2 topic pub --times 30 -r 10 /diff_drive_controller/cmd_vel \
  geometry_msgs/msg/TwistStamped \
  '{header: {frame_id: "base_link"}, twist: {linear: {x: -0.02}, angular: {z: 0.0}}}'
```

**Forventet resultat:** Roveren kjører tilbake samme vei. `odom.position.x`
reduseres.

### 5.3 Nullstill/stopp

```bash
ros2 topic pub --once /diff_drive_controller/cmd_vel \
  geometry_msgs/msg/TwistStamped \
  '{header: {frame_id: "base_link"}, twist: {linear: {x: 0.0}, angular: {z: 0.0}}}'
```

---

## 6. Test C – Svinging

### 6.1 Sving på stedet (til venstre)

**Terminal 2:**

```bash
# OBS: angular.z > ~0.2 tipper den tynne, lette roveren. Hold deg under 0.2.
ros2 topic pub --times 40 -r 10 /diff_drive_controller/cmd_vel \
  geometry_msgs/msg/TwistStamped \
  '{header: {frame_id: "base_link"}, twist: {linear: {x: 0.0}, angular: {z: 0.15}}}'
```

**Forventet resultat:**

- Venstre hjul spinner bakover, høyre spinner framover (eller motsatt,
  avhengig av koordinatsystem).
- I Gazebo roteres roveren rundt eget senter.
- `odom.pose.pose.orientation.z/w` endrer seg merkbart.

### 6.2 Kjør i en bue

```bash
# Buer til høyre mens den rygger — hold angular.z moderat (~0.15 rad/s)
ros2 topic pub --times 60 -r 10 /diff_drive_controller/cmd_vel \
  geometry_msgs/msg/TwistStamped \
  '{header: {frame_id: "base_link"}, twist: {linear: {x: 0.02}, angular: {z: -0.15}}}'
```

**Forventet resultat:** Høyre hjul spinner langsommere enn venstre.
Roveren tegner en smuul bue bakover-høyre.

---

## 7. Test D – Rocker-bogie-suspensjon og differensial

Dette er den viktigste testen. Vi kjører roveren mot trinn/rampe og
verifiserer at mekanismen:

1. **rocker-leddene beveger seg** når hjulene møter hinderet,
2. **differensial-koblingen holder** (`rocker_left + rocker_right ≈ 0`),
3. **bogie-leddene kompenserer** for forskjellig høyde på front/bakhjul
   på samme side.

### 7.1 Observer `/joint_states` live

**Terminal 3:**

```bash
ros2 topic echo /joint_states
```

Eller bare posisjon:

```bash
ros2 topic echo /joint_states --field position
```

**Hvile-posisjon (før du sender cmd_vel):**

- Alle hjul-joints står stille (velocity ≈ 0).
- `rocker_left_joint`, `rocker_right_joint` ≈ 0 rad (±0.01).
- `bogie_left/right_joint` ≈ 0 rad.
- `hengsel_diff_*` og `rocker_bogie_diff_joint` ≈ 0 rad.

### 7.2 Kjør inn i trinnet

I **Terminal 2**, sett roveren rett fram mot `step_obstacle`
(står ca. 10 cm foran spawn):

```bash
ros2 topic pub --times 100 -r 10 /diff_drive_controller/cmd_vel \
  geometry_msgs/msg/TwistStamped \
  '{header: {frame_id: "base_link"}, twist: {linear: {x: 0.02}, angular: {z: 0.0}}}'
```

**Forventet resultat mens frontre hjul klatrer over trinnet:**

- `rocker_left_joint` vinkel endres (f.eks. +0.15 rad).
- `rocker_right_joint` vinkel endres med **motsatt fortegn** (f.eks. -0.15 rad).
- Summen holder seg **nær 0** (vanligvis innenfor ±0.05 rad). Det er PD-
  pluggen (`RockerBogieDifferential`) som jobber i fysikk-loopen.
- `bogie_*_joint` svinger litt mens bogie-hjulene passerer kanten.

### 7.3 Plott suspensjonen (valgfritt)

**Terminal 3:**

```bash
ros2 run rqt_plot rqt_plot
```

Legg til i plottet:

- `/joint_states/position[X]` for `rocker_left_joint`
- `/joint_states/position[Y]` for `rocker_right_joint`

(X og Y avhenger av URDF-rekkefølgen – kopier navnene fra
`ros2 topic echo /joint_states --once`.)

**Forventet resultat:** De to kurvene er nær perfekt speilvendt rundt 0.

### 7.4 Kvantitativ differensial-sjekk

Kjør en enkel sanity-print:

```bash
# Terminal 3
ros2 topic echo /joint_states --once | \
  python3 -c "
import yaml, sys
d = yaml.safe_load(sys.stdin)
pos = dict(zip(d['name'], d['position']))
L = pos['rocker_left_joint']
R = pos['rocker_right_joint']
print(f'rocker_L={L:+.4f}  rocker_R={R:+.4f}  sum={L+R:+.4f}')"
```

**Forventet resultat:** `|sum| < 0.05 rad` i jevn bevegelse. Kort transient
opp mot 0.1-0.2 rad når hjulet treffer kanten er OK – plugin-en trekker
det raskt mot 0 igjen.

---

## 8. Test E – Passering av rampe og steiner

Kjør roveren sidelengs mot rampen eller over en av de små `rock_N`-modellene:

```bash
# Terminal 2: kjør framover 10 sekunder
ros2 topic pub --times 100 -r 10 /diff_drive_controller/cmd_vel \
  geometry_msgs/msg/TwistStamped \
  '{header: {frame_id: "base_link"}, twist: {linear: {x: 0.02}, angular: {z: 0.0}}}'
```

**Forventet resultat:**

- Roveren klatrer opp rampen uten å tippe eller snurre fast.
- Når den passerer én stein med et enkelt hjul, går `bogie_left_joint`
  (eller `bogie_right_joint`) opp, mens tilsvarende på motsatt side holder
  seg tilnærmet stille.
- Roverens `base_link` holder pitch-vinkelen relativt rolig sammenliknet
  med hjulene – det er poenget med hele rocker-bogie-designet.

---

## 9. Test F – Tastatur-teleop (interaktivt)

For fri kjøring:

```bash
sudo apt install -y ros-jazzy-teleop-twist-keyboard

# Terminal 2
# teleop_twist_keyboard publiserer Twist (ikke TwistStamped), så vi må enten
# (a) bruke stamped-mode på nyere teleop, eller (b) kjøre en enkel relay-node.
# Enklest i Jazzy: bruk stamped-parameteren og remap til diff_drive controller.
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args \
  -r /cmd_vel:=/diff_drive_controller/cmd_vel \
  -p stamped:=true \
  -p frame_id:=base_link
```

Tastebinding:

- `i` / `,` – fram / bak
- `j` / `l` – sving venstre / høyre
- `u` `o` `m` `.` – kombinert fram/sving
- `k` – stopp
- `q` / `z` – øk / reduser max hastighet

Sett fartene lavt (~0.02 m/s, 0.5 rad/s). Roveren er ~5 cm – høye verdier
gir kaotisk bevegelse.

---

## 9b. Test G – Sensorer (IMU + stereokamera + RGB-D)

Roveren har en IMU midt i basen, et stereokamera foran og et
RGB-D-kamera (depth) midt i det samme kamerahuset. Alle publiseres
fra Gazebo via `ros_gz_bridge` på ROS 2-topics.

### IMU

Sjekk at topicen finnes og oppfører seg fysisk fornuftig:

```bash
# Er topicen der?
ros2 topic list | grep imu
# -> /imu

# Se rådata (100 Hz). Stå stille = linear_acceleration ≈ (0, 0, 9.81).
ros2 topic echo /imu --once

# Rate-sjekk
ros2 topic hz /imu
# -> average rate ~ 100 Hz
```

Kjør deretter en bevegelsestest og se at IMU-en reagerer:

```bash
# Terminal A: logg vinkelhastighet z (gyro) og akselerasjon x
ros2 topic echo /imu --field angular_velocity.z
# Terminal B: drei roveren
ros2 topic pub --once /diff_drive_controller/cmd_vel \
  geometry_msgs/msg/TwistStamped \
  "{twist: {linear: {x: 0.0}, angular: {z: 0.15}}}"
```

**Forventet:** `angular_velocity.z` gir en klart positiv verdi under svingen
(når roveren roterer CCW), og faller tilbake til ~0 når du stopper.

Plot IMU-data pent i RViz: legg til en `Imu`-display og sett `Topic=/imu`.

### Stereokamera

Topics som bridges:

| Topic                             | Type                         | Oppløsning | Sim-rate |
|-----------------------------------|------------------------------|------------|----------|
| `/stereo/left/image_raw`          | `sensor_msgs/msg/Image`      | 320×240    | 15 Hz    |
| `/stereo/left/camera_info`        | `sensor_msgs/msg/CameraInfo` | –          | 15 Hz    |
| `/stereo/right/image_raw`         | `sensor_msgs/msg/Image`      | 320×240    | 15 Hz    |
| `/stereo/right/camera_info`       | `sensor_msgs/msg/CameraInfo` | –          | 15 Hz    |

Sjekk at de tikker (wall-clock-raten blir `15 × RTF`):

```bash
ros2 topic list | grep stereo
ros2 topic hz /stereo/left/image_raw
ros2 topic hz /stereo/right/image_raw
```

Visualiser i RViz:

1. `Add` → `By topic` → `/stereo/left/image_raw/Image`
2. `Add` → `By topic` → `/stereo/right/image_raw/Image`
3. Under `Global Options`, sett `Fixed Frame=base_link` (eller `odom`) slik at TF-oppslag virker.

Alternativ raskere visning (uten RViz):

```bash
# Paa Jazzy: ros-jazzy-rqt-image-view
ros2 run rqt_image_view rqt_image_view
# Velg topic /stereo/left/image_raw i dropdown
```

**Forventet:**
- Begge bilder vises i 320×240, 15 Hz simulert (≈ `15 × RTF` wall-clock).
- Når du drar roveren forover (`linear.x=0.02`) ruller scenen mot deg.
- Når du svinger, panorerer scenen.
- Venstre og høyre bilde har ~10 mm parallax (stereo-baseline).

### RGB-D / Depth-kamera

Én `rgbd_camera`-sensor publiserer alle fire topics samtidig:

| Topic                         | Type                          | Innhold                        |
|-------------------------------|-------------------------------|--------------------------------|
| `/depth_camera/image`         | `sensor_msgs/msg/Image`       | Farge (RGB8, 320×240)          |
| `/depth_camera/depth_image`   | `sensor_msgs/msg/Image`       | Dybde (32FC1, meter pr. piksel)|
| `/depth_camera/points`        | `sensor_msgs/msg/PointCloud2` | Farget 3D-punktsky             |
| `/depth_camera/camera_info`   | `sensor_msgs/msg/CameraInfo`  | Intrinsics                     |

Rate-sjekk:

```bash
ros2 topic hz /depth_camera/image
ros2 topic hz /depth_camera/depth_image
ros2 topic hz /depth_camera/points
```

Alle tre bør ligge rundt `15 × RTF` Hz.

Visualiser dybden og punktskyen:

```bash
# Depth-bilde (gråtoner: nær=mørkt, fjernt=lyst)
ros2 run rqt_image_view rqt_image_view /depth_camera/depth_image
```

I RViz:

1. `Fixed Frame = base_link`
2. `Add` → `By topic` → `/depth_camera/points/PointCloud2`
3. Sett `Size (m) = 0.01` og `Style = Points` eller `Flat Squares`.
4. Sett `Color Transformer = RGB8` for å se fargelagt punktsky, eller
   `AxisColor` for å farge etter Z (høyde).

**Forventet:**
- Depth-bildet viser bakken som en gradient fra mørkt (nært) til lyst
  (fjernt); hinder og steiner står som distinkte "hull" i gradienten.
- Punktskyen i RViz viser terreng, rampe og steiner som en farget 3D-
  mesh foran roveren.
- Verdier i `depth_image` er meter (sjekk med `ros2 topic echo
  /depth_camera/depth_image --field encoding` → `32FC1`).
- Punktene klippes ved `far=10 m` og ingenting nærmere enn `near≈0.1 m`
  (ogre2-grense).

### TF-sjekk for sensor-frames

```bash
# Skal eksistere alle disse:
ros2 run tf2_ros tf2_echo base_link imu_link
ros2 run tf2_ros tf2_echo base_link stereo_camera_link
ros2 run tf2_ros tf2_echo base_link stereo_left_optical_frame
ros2 run tf2_ros tf2_echo base_link stereo_right_optical_frame
ros2 run tf2_ros tf2_echo base_link depth_camera_optical_frame
```

---

## 10. Validerings-sjekkliste

Kryss av mens du tester. Alle skal være OK etter en normal run:

- [ ] `ros2 control list_controllers` viser begge kontrollere som `active`.
- [ ] RViz: alle links i `Displays`-panelet har `Transform OK` (grønt).
- [ ] `linear.x=0.02` → alle seks hjul roterer med ~2.7 rad/s i samme retning.
- [ ] `linear.x=-0.02` → samme fart, motsatt retning.
- [ ] `angular.z≠0, linear.x=0` → venstre og høyre hjul spinner motsatt vei.
- [ ] `odom.position.x` vokser/krymper i riktig retning.
- [ ] Ved kryssing av trinn/rampe: `rocker_left + rocker_right ≈ 0` (|sum|<0.05 rad).
- [ ] `bogie_*_joint` beveger seg synlig når bogie-hjul passerer en stein.
- [ ] Roveren tipper ikke, går ikke gjennom bakken, og oscillerer ikke ukontrollert.
- [ ] `/imu` publiseres i ~100 Hz og viser ~9.81 m/s² på z i ro.
- [ ] `/stereo/left/image_raw` og `/stereo/right/image_raw` publiseres i ~`15 × RTF` Hz.
- [ ] Stereo-bildene viser ulik parallax (baseline ~10 mm) for nære objekter.
- [ ] `/depth_camera/image`, `/depth_camera/depth_image`, `/depth_camera/points` og `/depth_camera/camera_info` publiseres i ~`15 × RTF` Hz.
- [ ] Depth-bildet viser gradient (nær = mørkt, fjernt = lyst); hinder trer tydelig frem.
- [ ] RViz PointCloud2-visning av `/depth_camera/points` viser farget 3D-geometri foran roveren.
- [ ] `tf2_echo base_link stereo_left_optical_frame` returnerer en gyldig transform.
- [ ] `tf2_echo base_link depth_camera_optical_frame` returnerer en gyldig transform.

---

## 11. Feilsøking

### 11.1 Roveren er usynlig / bare aksekors

Mesh-filer kan ikke lastes. Se etter i Terminal 1-loggen:

```
[Err] Unable to find file with URI [model://moonmapper_description/meshes/...]
```

Fiks: verifiser at hooken er riktig:

```bash
cat install/moonmapper_description/share/moonmapper_description/environment/moonmapper_description.dsv
# Skal inneholde:
#   prepend-non-duplicate;GZ_SIM_RESOURCE_PATH;share
```

Rebuild og re-source:

```bash
colcon build --packages-select moonmapper_description --symlink-install
source install/setup.bash
```

### 11.2 Roveren synker gjennom bakken

Spawn-høyden er for lav i forhold til kollisjonen. I `launch/gazebo_rover.launch.py`:

```python
DeclareLaunchArgument("spawn_z", default_value="0.05")
```

Øk til 0.08–0.10 hvis den fortsatt faller ned.

### 11.3 Roveren kjører ikke når jeg publiserer cmd_vel

Symptom: `ros2 topic pub ...` sier `Waiting for at least 1 matching subscription(s)...`
i det uendelige.

Sjekkliste:

1. **Topic-navn:** Riktig topic er `/diff_drive_controller/cmd_vel` (ikke
   `cmd_vel_unstamped` – den ble fjernet i Jazzy v4.39+).
2. **Meldingstype:** Den MÅ være `geometry_msgs/msg/TwistStamped`, ikke `Twist`:

   ```bash
   ros2 topic info /diff_drive_controller/cmd_vel -v
   # Skal vise: Type: geometry_msgs/msg/TwistStamped og Subscription count: 1
   ```

3. **Eksempel som virker:**

   ```bash
   ros2 topic pub --once /diff_drive_controller/cmd_vel \
     geometry_msgs/msg/TwistStamped \
     '{header: {frame_id: "base_link"}, twist: {linear: {x: 0.02}}}'
   ```

4. **Hardware-interfaces** skal være claimed (krever `ros2controlcli`):

   ```bash
   ros2 control list_hardware_interfaces
   # wheel_*_joint/velocity skal stå som [available] [claimed]
   ```

### 11.4 RViz sier "No transform from [hengsel_diff_L_link]" osv.

Det betyr at `/joint_states` ikke inneholder disse joint-ene.
Verifiser at `<ros2_control>` i `urdf/moonmapper_rover_gazebo.urdf.xacro`
inneholder `<state_interface>` for `hengsel_diff_L_joint`,
`hengsel_diff_R_joint` og `rocker_bogie_diff_joint`, og bygg på nytt.

### 11.5 Plugin laster ikke

Terminal 1 skulle si:

```
[Msg] [RockerBogieDifferential] Configured for model 'moonmapper'
```

Gjør den ikke det, sjekk at `libmoonmapper_rocker_bogie_differential.so`
finnes:

```bash
find install -name "libmoonmapper_rocker_bogie_differential.so"
# Skal ligge i install/moonmapper_description/lib/
```

Og at `GZ_SIM_SYSTEM_PLUGIN_PATH` peker dit (launch-filen setter dette
automatisk).

---

## 12. Filer å kjenne til

| Fil | Rolle |
|---|---|
| `urdf/moonmapper_rover.urdf.xacro` | Grunn-URDF: links, joints, inertial, visuelle meshes |
| `urdf/moonmapper_rover_gazebo.urdf.xacro` | Gazebo-wrapper: friksjon, `ros2_control`, plugin-lasting |
| `gz_plugins/src/RockerBogieDifferential.cc` | C++ system-plugin som PD-kontrollerer differensialen |
| `config/wheel_controllers.yaml` | `diff_drive_controller`-parametere (hjulradius, separasjon, grenser) |
| `config/ros_gz_bridge.yaml` | Broer `/clock` og `/tf` mellom Gazebo og ROS 2 |
| `worlds/rocker_bogie_test.sdf` | Testverden (bakke, trinn, rampe, steiner) |
| `launch/gazebo_rover.launch.py` | Orkestrerer Gazebo + RSP + bridge + spawners + RViz |

---

## 13. Nyttige snarveier

```bash
# Hopp inn i topic-graf (rqt)
ros2 run rqt_graph rqt_graph

# Live TF-tre
ros2 run tf2_tools view_frames
# Lager frames.pdf i gjeldende mappe

# Live visning av controllers og hardware
ros2 control list_controllers
ros2 control list_hardware_interfaces
```
