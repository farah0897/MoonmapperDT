# moonmapper_mapping

SLAM med **slam_toolbox** (online async) uten Nav2 — bygger `/map` fra `/scan`, `/odom` og `/tf`.

## Bygg pakken

```bash
cd ~/rover/simulasjon/moonmapper_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select moonmapper_mapping --symlink-install
source install/setup.bash
```

## Viktig: hvorfor `/map` tidligere «hang» uten data

`async_slam_toolbox_node` er en **LifecycleNode**. Med vanlig `Node()` i launch ble den **aldri aktivert** (ingen Configure/Activate), så den abonnerte ikke skikkelig og **publiserte ikke `/map`**.  
`slam_mapping.launch.py` følger nå samme mønster som `slam_toolbox/share/launch/online_async_launch.py` (**LifecycleNode** + **EmitEvent/RegisterEventHandler** for configure → activate).

## Avhengighet: slam_toolbox

```bash
ros2 pkg prefix slam_toolbox
```

Hvis kommandoen feiler:

```bash
sudo apt update
sudo apt install ros-jazzy-slam-toolbox
```

(Bytt `jazzy` med din ROS 2-distribusjon, f.eks. `humble`.)

## TF som må finnes

Før SLAM fungerer må kjeden være komplett:

- `odom` → `base_footprint` (diff_drive / relay)
- `base_footprint` → laser/depth-kamera frame som brukes i **`/scan.header.frame_id`** (typisk fra `depth_to_scan_node` / URDF)

Sjekk:

```bash
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 topic echo /scan --once
```

Noter `frame_id` på `/scan`. Deretter:

```bash
ros2 run tf2_ros tf2_echo base_footprint <din_scan_frame_id>
```

Hvis den siste mangler, må URDF/`robot_state_publisher` utvide TF (ikke «hacke» `frame_id` i `depth_to_scan_node` uten å fikse modellen).

## Kart-TF

Etter at `slam_toolbox` kjører skal `map` → `odom` publiseres av noden:

```bash
ros2 run tf2_ros tf2_echo map odom
```

## SLAM health check

Manuelt:

```bash
ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic echo /scan --once
ros2 topic echo /odom --once
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 param get /slam_toolbox scan_topic
ros2 param get /slam_toolbox map_frame
ros2 param get /slam_toolbox odom_frame
ros2 param get /slam_toolbox base_frame
ros2 lifecycle get /slam_toolbox
ros2 topic info /map -v
ros2 topic echo /map --once
ros2 node info /slam_toolbox
```

Eller script (etter `source install/setup.bash`):

```bash
bash $(ros2 pkg prefix moonmapper_mapping)/lib/moonmapper_mapping/slam_health_check.sh
```

For `/scan.header.frame_id` → sjekk TF `base_footprint` → den framen (se over).

## `/odom` og `base_frame`

Med standard MoonMapper diff_drive + relay er typisk:

- `header.frame_id`: `odom`
- `child_frame_id`: `base_footprint`

Da skal `slam_toolbox` sin `base_frame` være **`base_footprint`** (som i `config/slam_toolbox_online_async.yaml`).

## Testprosedyre

**Terminal 1 — sim (startes separat, ikke fra mapping-launch):**

```bash
cd ~/rover/simulasjon/moonmapper_ws
source install/setup.bash
ros2 launch moonmapper_bringup sim_rover_clean.launch.py
```

**Terminal 2 — autonomi + SLAM + coverage:**

```bash
cd ~/rover/simulasjon/moonmapper_ws
source install/setup.bash
ros2 launch moonmapper_mapping autonomy_slam_test.launch.py
```

**Samme som over, men start innebygd Steg 4-RViz** (LaserScan `/scan`, kart `/map` og `/coverage/grid`):

```bash
ros2 launch moonmapper_mapping autonomy_slam_test.launch.py rviz:=true
```

Unngå **to** RViz-vinduer: start sim med `use_rviz:=false` når du bruker `rviz:=true` her, eller la sim beholde RViz og utelat `rviz:=true`.

**Kun RViz med Steg 4-oppsett** (når noder allerede kjører i andre terminaler):

```bash
ros2 launch moonmapper_mapping slam_rviz.launch.py
```

**Kun SLAM** (når `/scan`, `/odom`, `/tf` allerede kjører):

```bash
ros2 launch moonmapper_mapping slam_mapping.launch.py
```

**Terminal 3 — observasjon:**

```bash
source ~/rover/simulasjon/moonmapper_ws/install/setup.bash
ros2 topic echo /map --once
ros2 topic hz /map
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 topic echo /scan --once
ros2 topic echo /obstacle/current_state
ros2 topic echo /coverage/percent
```

## Godkjenning Steg 4

- `ros2 lifecycle get /slam_toolbox` viser **`active`** (etter noen sekunder fra launch).
- `ros2 topic echo /map --once` gir `nav_msgs/msg/OccupancyGrid` (ikke heng).
- `ros2 topic hz /map` viser publiseringsrate.
- `ros2 run tf2_ros tf2_echo map odom` fungerer.
- RViz med `moonmapper_slam.rviz`: **SLAM Map** får data (ikke «No map received»); Fixed frame kan settes til **`map`** når TF finnes.
- Roboten kjører fortsatt med reactive avoidance + safety (uendret).

## RViz

**Viktig:** RViz som starter fra `gazebo_rover` / `moonmapper_description` (`moonmapper.rviz`) har **ikke** LaserScan eller SLAM-kart — derfor ser du ofte bare `base_footprint`/`odom` og ingen `/scan` i «Add display».

Bruk eget oppsett for autonomi + SLAM (etter `source install/setup.bash` og `colcon build`):

```bash
ros2 launch moonmapper_mapping slam_rviz.launch.py
```

eller start RViz sammen med autonomi (`rviz:=true`, se testprosedyre over). Konfigurasjonsfil:

```bash
rviz2 -d $(ros2 pkg prefix moonmapper_mapping)/share/moonmapper_mapping/rviz/moonmapper_slam.rviz
```

- **Fixed frame:** `odom` i denne filen (fungerer før og etter SLAM). Når `slam_toolbox` kjører og `map`→`odom` finnes, kan du bytte til **`map`** i *Global Options* for klassisk SLAM-visning.
- **LaserScan:** `/scan` (krever at Terminal 2 med `autonomy_slam_test` — eller minst `depth_to_scan` — kjører).
- **SLAM Map:** `/map` (krever at `slam_toolbox` kjører uten feil).
- **Coverage:** `/coverage/grid` (frame `odom`).

### `tf2_echo` «command not found»

ROS 2 har ikke kommandoen `tf2_echo` alene. Bruk:

```bash
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo map odom
```

### Hvorfor «map» ikke finnes i Fixed Frame-listen

Rammen **`map`** dukker først opp når **`slam_toolbox`** publiserer TF **`map` → `odom`**. Uten kjørende SLAM-noder finnes ikke `map`. Da er **`odom`** (eller `base_footprint`) riktig valg.

### Hvorfor `/scan` ikke vises i RViz «By topic»

Da finnes **ingen publisher** på `/scan` (typisk: bare sim + RViz startet, **ikke** `autonomy_slam_test.launch.py`). Kjør:

```bash
ros2 topic list | grep -E '^/scan$'
ros2 node list | grep -E 'depth_to_scan|slam_toolbox'
```


## Steg 5: Lagre kart (5A) og localization (5B)

### Viktig: to typer kartfiler

| Type | Verktøy | Filer | Bruk |
|------|---------|-------|------|
| **OccupancyGrid** | `nav2_map_server` **map_saver_cli** | `.yaml` + `.pgm` | Visning, fremtidig AMCL/`map_server`, bildevisning |
| **Posegraf (slam_toolbox)** | tjenesten `/slam_toolbox/serialize_map` | `.posegraph` + `.data` | **`localization_slam_toolbox_node`** (Steg 5B) |

`slam_toolbox` localization leser **ikke** Nav2-`.yaml` direkte; den trenger serialiseringen. Scriptet `save_current_map.sh` lager **begge** med samme stamnavn (f.eks. `moonmapper_test_map` → `moonmapper_test_map.yaml`, `.pgm`, `.posegraph`, `.data`).

### Avhengighet: nav2_map_server (kun map_saver_cli)

```bash
ros2 pkg prefix nav2_map_server
```

Hvis den feiler:

```bash
sudo apt update
sudo apt install ros-jazzy-nav2-map-server
```

(Bytt `jazzy` med din ROS 2-distribusjon.)

Kartfiler lagres standard under **installert pakke-del**:

`$(ros2 pkg prefix moonmapper_mapping)/share/moonmapper_mapping/maps/`

Valgfritt: sett `MOONMAPPER_MAPS_DIR` til en annen katalog. Valgfritt: `MOONMAPPER_USE_SIM_TIME=false` hvis du lagrer uten sim-klokke.

### Steg 5A — Bygg kart i mapping, deretter lagre

**Terminal 1 — sim:**

```bash
cd ~/rover/simulasjon/moonmapper_ws
source install/setup.bash
ros2 launch moonmapper_bringup sim_rover_clean.launch.py
```

**Terminal 2 — mapping + autonomi:**

```bash
source install/setup.bash
ros2 launch moonmapper_mapping autonomy_slam_test.launch.py
```

Kjør roboten til `/map` ser bra ut.

**Terminal 3 — lagre:**

```bash
source install/setup.bash
bash "$(ros2 pkg prefix moonmapper_mapping)/lib/moonmapper_mapping/save_current_map.sh" moonmapper_test_map
```

### Padde kart for Nav2 (`moonmapper_test_map_padded`)

For Nav2-baseline v2 brukes et **padded** kart (minst **2 m** fri kant rundt eksisterende `.yaml`/`.pgm`) slik at `global_costmap` og sensoren ikke faller utenfor rutenettet når roboten kjører litt utenfor det opprinnelige kartrektangelet.

```bash
source install/setup.bash
ros2 run moonmapper_mapping pad_occupancy_map.py \
  --input-yaml "$(ros2 pkg prefix moonmapper_mapping)/share/moonmapper_mapping/maps/moonmapper_test_map.yaml" \
  --output-dir "$(ros2 pkg prefix moonmapper_mapping)/share/moonmapper_mapping/maps" \
  --output-stem moonmapper_test_map_padded \
  --padding-m 2.0
```

Skriv gjerne til **`src/moonmapper_mapping/maps/`** under utvikling og kjør `colcon build --packages-select moonmapper_mapping`, eller skriv direkte til `install/.../share/moonmapper_mapping/maps/` for rask test uten ombygg (symlink-install).

Etter `colcon build` finnes kart også i **kildekatalog** `src/moonmapper_mapping/maps/` bare hvis du kopierer dit manuelt eller peker `MOONMAPPER_MAPS_DIR` dit; standard er **share/maps** under install.

**Godkjenning Steg 5A**

- `moonmapper_test_map.yaml` og `moonmapper_test_map.pgm` finnes.
- YAML `image:` peker på samme navn som `.pgm`.
- `resolution` i yaml er **0.05** (samme som SLAM-parameter `resolution`).
- PGM kan åpnes i en bildevisning.
- `moonmapper_test_map.posegraph` og `moonmapper_test_map.data` finnes (kreves for Steg 5B).

### Steg 5B — Localization mot lagret kart (uten Nav2 stack)

Sim startes fortsatt separat. Launch starter **ikke** Gazebo, Nav2, AMCL eller EKF.

**Terminal 1 — sim** (som over).

**Terminal 2 — localization + depth + reactive + safety:**

```bash
source install/setup.bash
ros2 launch moonmapper_mapping localization_test.launch.py \
  map_file:=/full/path/to/moonmapper_test_map.yaml
```

**RViz med Fixed Frame `map` (anbefalt for denne testen):**

```bash
ros2 launch moonmapper_mapping localization_test.launch.py \
  map_file:=/full/path/to/moonmapper_test_map.yaml \
  rviz:=true
```

Alternativt manuelt (etter `source install/setup.bash`):

```bash
rviz2 -d $(ros2 pkg prefix moonmapper_mapping)/share/moonmapper_mapping/rviz/moonmapper_localization.rviz
```

Konfigurasjonen `rviz/moonmapper_localization.rviz` har **Global Options → Fixed Frame = map**, rutenett i `map`, **RobotModel** (`robot_description` fra sim), **TF**, **LaserScan** `/scan`, **SLAM Map** `/map`, og verktøyet **2D Pose Estimate** (`/initialpose`) hvis roboten ikke ligger riktig ved oppstart.

`map_file` kan være full sti til **`.yaml`** fra map_saver; launch stripper suffiks og setter `map_file_name` til **stam** som matcher `.posegraph`/`.data`. Standard `map_file` (hvis du ikke overstyrer) er den delte stien til `maps/moonmapper_slam_map.yaml` i pakken — den finnes ikke før du har lagret et kart med det navnet.

Standard oppstartspose: **`map_start_pose: [0.0, 0.0, 0.0]`** i `config/slam_toolbox_localization.yaml` (kart lastes med gitt pose i kartet; juster med **2D Pose Estimate** i RViz om roboten ikke starter i origo relativt til kartet).

**Vanlig feil:** `map_file` peker feil — sti må gå til **`.../maps/dittnavn.yaml`**, ikke inn under `rviz/moonmapper_localization.rviz/...`. Da finnes ikke `.posegraph`/`.data`, kartet lastes ikke, og du ser et **nytt** kart som vokser (som ved mapping).

**Terminal 3 — validering:**

```bash
ros2 topic echo /map --once
ros2 topic hz /map
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 topic echo /scan --once
ros2 topic echo /cmd_vel
ros2 topic echo /obstacle/current_state
ros2 lifecycle get /slam_toolbox
```

**RViz:** Fixed Frame = **`map`**, vis **Map** `/map`, **RobotModel**, **TF**, **LaserScan** `/scan` (bruk `moonmapper_localization.rviz` eller `rviz:=true` på launch over).

**Godkjenning Steg 5B**

- Lagret kart vises (raster fra posegraf; skal ikke «vekse» som ved fri mapping).
- `map` → `odom` og `odom` → `base_footprint` finnes.
- Roboten ligger konsistent på kartet; reactive avoidance + safety kjører uendret fra `moonmapper_autonomy`.
- Ingen Nav2-navigator-/controller-/bt_navigator-noder startes fra denne launch.

**Vanlige feil (localization)**

1. **Feil `map_file`-sti** — f.eks. `.../rviz/moonmapper_localization.rviz/moonmapper_test_map.yaml` (`.rviz` er en **fil**, ikke en mappe). Da finnes ikke posegrafen, og du får et **nytt** voksende kart som ved mapping. Bruk `.../maps/moonmapper_test_map.yaml` der `save_current_map.sh` skrev filene.
2. **Mangler `.posegraph`/`.data`** — du har bare kjørt map_saver manuelt; kjør `save_current_map.sh` (steg 2 kaller `serialize_map`) mens mapping-noden kjører.
3. **`map_file_name` peker feil** — bruk full sti til `.yaml` som matcher filene som ble lagret i samme `maps/`-katalog.
4. **Lifecycle ikke `active`** — som Steg 4; sjekk `ros2 lifecycle get /slam_toolbox`.
5. **`use_sim_time` feil** — sim må bruke `/clock`; launch bruker `use_sim_time:=true` som standard.
6. **Feil startpose** — bruk **2D Pose Estimate** i RViz (`/initialpose`) hvis roboten ikke matcher kartet ved oppstart.

## Vanlige feil hvis `/map` ikke kommer

1. **Lifecycle ikke aktiv** — skal være løst i nåværende `slam_mapping.launch.py`; sjekk `ros2 lifecycle get /slam_toolbox` = `active`.
2. **`slam_toolbox` ikke installert** — se avhengighet over.
3. **Ingen `/scan`** — start `depth_to_scan_node` (via `autonomy_slam_test` eller `depth_reactive_avoidance`).
4. **TF mangler** — `tf2_echo` over feiler; fiks URDF / `robot_state_publisher` / Gazebo-broer.
5. **`use_sim_time` mismatch** — sim og noder må ha `use_sim_time:=true` når du bruker Gazebo `/clock`.
6. **`frame_id` på `/scan` uten transform til `base_footprint`** — SLAM får ikke lagt skann inn i kartet.
7. **Feil RViz-konfig** — standard `moonmapper.rviz` har ikke `/scan` eller `/map`; bruk `moonmapper_slam.rviz` over.
8. **Kun Terminal 1 (sim) startet** — uten Terminal 2 finnes ikke `/scan` eller aktiv SLAM-stack.

## Hva som *ikke* startes her

Ingen Nav2 **planlegging/navigasjon** (`planner_server`, `controller_server`, `bt_navigator`, AMCL). `nav2_map_server` brukes kun som **CLI** (`map_saver_cli`) for å skrive `.yaml`/`.pgm` i Steg 5A. Ingen EKF/UWB fra disse launch-filene.

## Steg 6: Enkel goal follower (uten Nav2, uten reactive)

**Launch:** `goal_localization_test.launch.py` — localization + `depth_scan_safety` + `simple_goal_follower_node` (ingen `reactive_avoidance_node`).

**Kommandokjede og fortegn:** På **`/cmd_vel_raw`** og **`/cmd_vel`** betyr `linear.x > 0` kjør framover med **fysisk front**, og `angular.z > 0` / `< 0` er **venstre** / **høyre** sving. `safety_obstacle_node` endrer **ikke** fortegn på `angular.z`. `cmd_vel_odom_relay` skalerer inn mot `diff_drive_controller` med `cmd_linear_x_sign` / `cmd_angular_z_sign` (standard **+1** / **+1** etter fysikk-baseline; midlertidig **-1** på linear kun hvis URDF/controller fortsatt er speilet). Utdatert `invert_cmd_vel_twist:=true` gir advarsel og legacy (neger **begge** — unngås i sim).

**Terminal 1 — sim:**

```bash
ros2 launch moonmapper_bringup sim_rover_clean.launch.py
```

**Sign-test (etter `colcon build` og `source install/setup.bash`)**

Test **A** — lineær retning (kun sim, ingen goal-launch):

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.03}, angular: {z: 0.0}}" -r 10
```

Forventet: roboten kjører **framover** med fysisk front. Baklengs → fiks URDF hjulakser / L–R i `diff_drive_controller.yaml` (unngå permanent `cmd_linear_x_sign:=-1`).

Test **B** — rotasjonsretning:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.25}}" -r 10
```

Forventet: roboten roterer **venstre** i kart/RViz. Feil vei → juster `cmd_angular_z_sign`.

Test **C** — safety snur ikke `angular.z`: start `goal_localization_test` (Terminal 2 under), sett mål eller vent på rotasjon, deretter:

```bash
ros2 topic echo /cmd_vel_raw --once
ros2 topic echo /cmd_vel --once
```

`angular.z` skal ha **samme fortegn** på begge topic (når scan er gyldig; ved `no_scan` er `/cmd_vel` null).

**Terminal 2 — localization + goal follower + safety:**

```bash
cd /home/farr97/rover/simulasjon/moonmapper_ws
source install/setup.bash
ros2 launch moonmapper_mapping goal_localization_test.launch.py \
  map_file:=/home/farr97/rover/simulasjon/moonmapper_ws/install/moonmapper_mapping/share/moonmapper_mapping/maps/moonmapper_test_map.yaml \
  rviz:=true \
  forward_speed:=0.12 \
  min_forward_speed:=0.035 \
  angular_speed:=0.45 \
  safety_front_stop_distance:=0.55
```

(Portabelt alternativ: `map_file:=$(ros2 pkg prefix moonmapper_mapping)/share/moonmapper_mapping/maps/moonmapper_test_map.yaml`.)

I RViz: **2D Goal Pose** (topic `/goal_pose` i `moonmapper_localization.rviz`). Kjede: `simple_goal_follower_node` (rotate-then-drive, `output_cmd_topic` standard **`/cmd_vel_raw`**) → `safety_obstacle_node` → `/cmd_vel` via `goal_localization_test.launch.py`.

**Terminal 3 — echo:**

```bash
source install/setup.bash
ros2 topic echo /goal_follower/state
ros2 topic echo /goal_follower/heading_error
ros2 topic echo /goal_follower/drive_allowed
ros2 topic echo /cmd_vel_raw
ros2 topic echo /cmd_vel
```

**Goal-follower test (streng sekvens):**

Test **A** — mål rett foran: forvent `DRIVE_TO_GOAL`, `drive_allowed: true`, `linear.x` > 0 på `/cmd_vel_raw`, robot kjører mot punktet.

Test **B** — mål bak roboten: forvent først `ROTATE_TO_GOAL`, `drive_allowed: false`, `linear.x` = 0, `angular.z` ≠ 0; når `|heading_error|` < ca **0,22** rad går den til `DRIVE_TO_GOAL` og `linear.x` > 0.

Test **C** — `/cmd_vel_raw` fra goal follower har aldri negativ `linear.x` (`ros2 topic echo /cmd_vel_raw`).

**Forventet etter kalibrering:** `heading_error` nær **0** når målet er foran (med `base_forward_yaw_offset_rad ≈ π`). Ved stor vinkel-feil under kjøring går tilstanden tilbake til **ROTATE** (grense `drive_stop_angle_rad`).

**Sim / depth→scan:** `goal_localization_test` har som standard `safety_front_stop_distance:=0.55`, `safety_front_angle_deg:=45`, `safety_scan_timeout_sec:=0.6`, `safety_allow_reverse_when_blocked:=true` (overstyr med launch-argumenter ved behov). `safety_obstacle_node` publiserer **`/obstacle/front_min`** og **`/obstacle/current_state`**.

**Yaw-kalibrering (uten URDF-endring):** `base_forward_yaw_offset_rad` (standard **π** i `goal_localization_test`) roterer den interne «frem»-retningen brukt i heading-feil: `robot_forward_yaw = normalize(robot_yaw_raw + offset)`. Juster offset ved behov. **`linear_sign`** / **`angular_sign`** (±1) flippes kun hvis fysisk kjøreretning motsvarer ikke `cmd_vel`.

## Steg 7: Goal + enkel hindringsunnamanøvre (uten Nav2, uten reactive)

**Launch:** `goal_obstacle_localization_test.launch.py` — localization + `depth_to_scan_node` + `simple_goal_follower_node` → **`/cmd_vel_goal`** → `goal_obstacle_avoidance_node` → **`/cmd_vel_raw`** → `safety_obstacle_node` → **`/cmd_vel`**. Ingen `reactive_avoidance_node`, ingen Nav2.

### Hastighet, safety og hindring (launch-argumenter)

Alle har standardverdier i launch-filen; du trenger ikke sette dem for å kjøre. Typiske justeringer:

| Argument | Standard | Kort forklaring |
|----------|----------|-------------------|
| `forward_speed` | `0.12` | Maks lineær frem hastighet hos goal follower (m/s). |
| `min_forward_speed` | `0.035` | Minimum frem hastighet. |
| `angular_speed` | `0.45` | Maks / roter hastighet (rad/s) hos goal follower. |
| `slow_radius` | `0.70` | Avstand der goal follower begynner å dempe frem hastighet (m). |
| `safety_stop_distance` | `0.55` | Safety: stopp `linear.x` foran hvis `front_min` &lt; dette (m). `angular.z` slippes gjennom. |
| `avoid_start_distance` | `0.90` | Hindringsmodus: start unnamanøver når front &lt; dette (m). |
| `avoid_clear_distance` | `1.20` | Hindringsmodus: anse front «klar» når &gt; dette (m). |

`goal_localization_test.launch.py` har `forward_speed`, `min_forward_speed`, `angular_speed`, `slow_radius` og safety-argumentene (`safety_front_stop_distance` osv.), men ikke `avoid_*` (ingen goal-obstacle-node der).

**Terminal 1 — sim:**

```bash
ros2 launch moonmapper_bringup sim_rover_clean.launch.py
```

**Terminal 2:**

```bash
source install/setup.bash
ros2 launch moonmapper_mapping goal_obstacle_localization_test.launch.py \
  map_file:=/home/farr97/rover/simulasjon/moonmapper_ws/install/moonmapper_mapping/share/moonmapper_mapping/maps/moonmapper_test_map.yaml \
  rviz:=true \
  forward_speed:=0.12 \
  angular_speed:=0.45 \
  safety_stop_distance:=0.55 \
  avoid_start_distance:=0.90 \
  avoid_clear_distance:=1.20
```

**Terminal 3 — echo:**

```bash
source install/setup.bash
ros2 topic echo /goal_follower/state
ros2 topic echo /goal_avoidance/state
ros2 topic echo /goal_avoidance/front_min
ros2 topic echo /goal_avoidance/left_min
ros2 topic echo /goal_avoidance/right_min
ros2 topic echo /goal_avoidance/selected_turn
ros2 topic echo /goal_avoidance/active
ros2 topic echo /cmd_vel_goal
ros2 topic echo /cmd_vel_raw
ros2 topic echo /cmd_vel
```

**Test A — fri vei:** Mål foran uten hindring → `goal_follower` i **DRIVE_TO_GOAL**, `goal_avoidance` **CLEAR**, `/cmd_vel_raw` følger `/cmd_vel_goal`, `/cmd_vel` følger `/cmd_vel_raw` (safety pass-through).

**Test B — hindring foran (detour):** Forvent sekvens **BRAKE** → **TURN_AWAY_LEFT/RIGHT** (kun rotasjon) → **ARC_LEFT/RIGHT** (`/cmd_vel_raw` med `linear.x` > 0 og `angular.z` ≠ 0, bue) → **RECOVER_TO_GOAL** (myk innfasing av goal-vinkel) → **CLEAR**. `active` er **true** under unnamanøver. Ved svært nær hindring kan **EMERGENCY** vises (kun rotasjon til `front_min` > `turn_clear_distance`).

**Test C — ingen /scan:** `goal_avoidance` = **NO_SCAN**, `/cmd_vel_raw` og `/cmd_vel` null.

## Steg 8: Waypoint-misjon (uten Nav2)

**Launch:** `waypoint_mission_test.launch.py` — inkluderer hele `goal_obstacle_localization_test`-stacken pluss `waypoint_mission_node`, som leser waypoints fra YAML og publiserer sekvensielt til `/goal_pose` til `simple_goal_follower` rapporterer **REACHED**.

**Bygg:**

```bash
cd ~/rover/simulasjon/moonmapper_ws
colcon build --packages-select moonmapper_mapping --symlink-install
source install/setup.bash
```

**Terminal 1 — sim:**

```bash
ros2 launch moonmapper_bringup sim_rover_clean.launch.py
```

**Terminal 2:**

```bash
source install/setup.bash
ros2 launch moonmapper_mapping waypoint_mission_test.launch.py \
  map_file:=/home/farr97/rover/simulasjon/moonmapper_ws/install/moonmapper_mapping/share/moonmapper_mapping/maps/moonmapper_test_map.yaml \
  waypoints_file:=/home/farr97/rover/simulasjon/moonmapper_ws/install/moonmapper_mapping/share/moonmapper_mapping/config/missions/test_waypoints.yaml \
  rviz:=true \
  auto_start:=true \
  loop:=false
```

**Terminal 3 — echo:**

```bash
source install/setup.bash
ros2 topic echo /mission/state
ros2 topic echo /mission/current_waypoint_index
ros2 topic echo /goal_follower/state
ros2 topic echo /goal_follower/distance_to_goal
ros2 topic echo /goal_pose
ros2 topic echo /cmd_vel
```

**Forventet:** `/mission/state` går **LOAD_WAYPOINTS** → **SEND_GOAL** (kort) → **WAIT_FOR_REACHED** → … → **DONE** etter siste punkt. Hindring underveis håndteres fortsatt av `goal_obstacle_avoidance_node`.

## Steg 9: Coverage-misjon / lawnmower (uten Nav2)

**Hva:** `coverage_mission_node` genererer en sikksakk-rute over et rektangel (parametre `area_*`, `lane_spacing`), publiserer hele banen som **`nav_msgs/Path`** på `/coverage_mission/generated_path`, og sender hvert hjørne sekvensielt som **`geometry_msgs/PoseStamped`** på `/goal_pose` (samme kontrakt som waypoint-misjon). `simple_goal_follower` + `goal_obstacle_avoidance_node` + `safety_obstacle_node` er uendret.

**Topics (utvalg):**

| Retning | Topic | Type |
|--------|--------|------|
| Ut | `/goal_pose` | `geometry_msgs/PoseStamped` |
| Ut | `/coverage_mission/generated_path` | `nav_msgs/Path` |
| Ut | `/coverage_mission/state` | `std_msgs/String` |
| Ut | `/coverage_mission/current_index`, `.../waypoint_count` | `std_msgs/Int32` |
| Ut | `/coverage_mission/active` | `std_msgs/Bool` |
| Inn | `/goal_follower/state`, `/goal_follower/distance_to_goal` | `std_msgs/String`, `Float32` |

**Launch:** `coverage_mission_test.launch.py` — samme stack som `waypoint_mission_test`, men med `coverage_mission_node` og RViz-profil `moonmapper_coverage_mission.rviz` (Path + kart + scan + TF + 2D Goal).

**Terminal 1 — sim:**

```bash
ros2 launch moonmapper_bringup sim_rover_clean.launch.py
```

**Terminal 2:**

```bash
source install/setup.bash
ros2 launch moonmapper_mapping coverage_mission_test.launch.py \
  map_file:=/home/farr97/rover/simulasjon/moonmapper_ws/install/moonmapper_mapping/share/moonmapper_mapping/maps/moonmapper_test_map.yaml \
  rviz:=true \
  auto_start:=true \
  loop:=false \
  forward_speed:=0.12 \
  angular_speed:=0.45 \
  safety_stop_distance:=0.55 \
  avoid_start_distance:=0.90 \
  avoid_clear_distance:=1.20
```

Valgfritt justere dekningsområde: `area_min_x:=...`, `area_max_x:=...`, `area_min_y:=...`, `area_max_y:=...`, `lane_spacing:=0.5`. Valgfritt tids-/toleranse-parametre til `coverage_mission_node`: `coverage_reached_tolerance`, `waypoint_hold_time` (koblet til `wait_after_reached_sec`), `publish_interval` (koblet til `goal_publish_period_sec`). `coverage_mission.launch.py` er et alias med samme argumenter som `coverage_mission_test.launch.py`.

**Terminal 3 — echo (én kommando av gangen):**

```bash
source install/setup.bash
ros2 topic echo /coverage_mission/state
ros2 topic echo /coverage_mission/current_index
ros2 topic echo /coverage_mission/waypoint_count
ros2 topic echo /goal_follower/state
ros2 topic echo /goal_follower/distance_to_goal
ros2 topic echo /cmd_vel
ros2 topic echo /cmd_vel_goal
ros2 topic echo /cmd_vel_raw
```

**Akseptanse (Steg 9):** Noden starter med `auto_start:=true`; `/coverage_mission/state` går **GENERATE_PATH** → **SEND_GOAL** → **WAIT_FOR_REACHED** → … → **DONE**. RViz viser sikksakk på **Coverage path**. Minst fire mål uten manuell 2D Goal; hindring foran håndteres av `goal_obstacle_avoidance_node`. `waypoint_mission_test.launch.py` og `goal_obstacle_localization_test.launch.py` fungerer fortsatt (sistnevnte har argumentet `rviz_config` med standard `moonmapper_localization.rviz`).

## Debug (hastighet, cmd_vel, TF)

Kjør én av gangen etter `source install/setup.bash`:

```bash
ros2 topic echo /cmd_vel_raw --once
ros2 topic echo /cmd_vel --once
ros2 topic echo /obstacle/front_min --once
ros2 topic echo /goal_follower/state --once
ros2 topic hz /scan
ros2 topic hz /odom
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_footprint
```

**Nav2-baseline** (`moonmapper_nav2`, se pakke-README):

```bash
ros2 node list | grep nav2
ros2 lifecycle nodes
ros2 topic echo /plan --once
ros2 topic echo /cmd_vel_raw --once
ros2 topic echo /cmd_vel --once
```

## Steg 10: Nav2-baseline (separat test)

Full stack med AMCL, planner, controller og BT ligger i pakken **`moonmapper_nav2`**. Nav2 skriver til **`/cmd_vel_raw`**; `nav2_bringup_test.launch.py` starter `safety_obstacle_node` som leser `/cmd_vel_raw` og publiserer til **`/cmd_vel`**. **Ikke** kjør Nav2 samtidig med `goal_obstacle_localization_test`, `coverage_mission*.launch.py` eller `waypoint_mission_test` (samme topics uten mux).

```bash
source install/setup.bash
ros2 launch moonmapper_nav2 nav2_bringup_test.launch.py rviz:=true
```

Standard er **`moonmapper_test_map_padded.yaml`** (Nav2 v2). Overstyr kart med `map_file:=$(ros2 pkg prefix moonmapper_mapping)/share/moonmapper_mapping/maps/moonmapper_test_map.yaml` ved behov.

Mer detaljer: `src/moonmapper_nav2/README.md`.

## Status (hva som er godkjent / ikke ennå)

| Område | Status |
|--------|--------|
| Steg 4 — online mapping (`async_slam_toolbox_node`), `/map`, `map`→`odom` | Forventet godkjent |
| Steg 5A — lagre `.yaml`/`.pgm` + posegraf via script | Implementert |
| Steg 6 — `/goal_pose` → goal follower → `/cmd_vel_raw` → safety | Implementert (`goal_localization_test.launch.py`) |
| Steg 7 — goal → `/cmd_vel_goal` → obstacle avoid → `/cmd_vel_raw` → safety | Implementert (`goal_obstacle_localization_test.launch.py`) |
| Steg 8 — waypoint-misjon (YAML → `/goal_pose`, samme obstacle-stack) | Implementert (`waypoint_mission_test.launch.py`) |
| Steg 9 — coverage / lawnmower (rektangel → Path + `/goal_pose`) | Implementert (`coverage_mission_test.launch.py`, alias `coverage_mission.launch.py`) |
| Steg 10 — Nav2-baseline (separat test, `/cmd_vel_raw` → safety → `/cmd_vel`) | Baseline i `moonmapper_nav2` (`nav2_bringup_test.launch.py`, `nav2_localization_test.launch.py`) |
| AMCL / `map_server` som del av Steg 10 (Nav2) | Ja, via `nav2_bringup`; Steg 5B bruker fortsatt `slam_toolbox` uten Nav2 |
| EKF / UWB | **Ikke** fra denne pakken |
