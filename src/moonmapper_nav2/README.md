# moonmapper_nav2

Separat **Nav2-baseline** for MoonMapper. Den erstatter **ikke** den fungerende `simple_goal_follower` + `goal_obstacle_avoidance` + `safety`-kjeden; den er ment som egen test med AMCL + kart + planner/controller.

## Viktig

- **Ikke** kjør `nav2_bringup_test.launch.py` eller `nav2_coverage_mission_test.launch.py` samtidig med `goal_obstacle_localization_test`, `coverage_mission*.launch.py` eller `waypoint_mission` (samme `/cmd_vel_raw`, `/cmd_vel` og `/scan`).
- Nav2 publiserer hastighet til **`/cmd_vel_raw`** (se `config/nav2_params_moonmapper.yaml` → `collision_monitor.cmd_vel_out_topic`). `safety_obstacle_node` i bringup/mission-launch leser `/cmd_vel_raw` og publiserer til **`/cmd_vel`**. Når hindring er **clear**, speiler sikkerhetsnoden Nav2-hastighet; den stopper eller begrenser ved nær hindring.
- Standardkart for Nav2-baseline **v2** er **`moonmapper_test_map_padded.yaml`** (2 m fri kant rundt opprinnelig kart — genereres med `pad_occupancy_map.py`). Overstyr med `map_file:=...` ved behov.

## Nav2 coverage / waypoint mission (NavigateThroughPoses)

Dette er et **nytt isolert steg**: `nav2_mission_client_node` sender flere mål i `map`-frame via Nav2-actionen **`nav2_msgs/action/NavigateThroughPoses`** (standard action-navn **`/navigate_through_poses`**). Nav2 sin **BT-navigator**, **planner** og **controller** styrer banen; det er **ikke** `simple_goal_follower`.

**Forskjell fra custom coverage/waypoint:** Custom-stack (`coverage_mission_node` / waypoint-misjon) følger egne punkter med egen logikk og ofte `simple_goal_follower`. Nav2-misjonen bruker **samme punkter i prinsippet**, men **global/lokal costmap**, **planner** (NavFn m.fl.) og **controller** (MPPI) løser banen og hastighet innenfor Nav2.

**Merk om waypoint-parameterfil:** ROS 2 RCL-parameter-YAML støtter **ikke** `waypoints:` som liste av `[x,y,yaw]`-lister. Bruk **flat** liste med tre og tre tall (se `config/nav2_coverage_waypoints.yaml`). Noden støtter også nested lister dersom du setter parametere programmatisk.

Misjonsnoden venter på **`/bt_navigator` lifecycle ACTIVE** og TF **`map`→`base_footprint`** når `mission_frame` er **`map`** (standard). Med **`mission_frame:=odom`** ventes **`odom`→`base_footprint`**; da brukes **`frame_id:=odom`** på alle mål, og kart-/AMCL-avhengigheter skrus av (se odom-test under). I **map**-modus kreves **N påfølgende vellykkede TF-oppslag** (`localization_tf_stable_count`, standard **5**) før mål sendes, for å unngå «ustabil» lokalisering rett etter 2D Pose Estimate. Ved avvist mål («action server inactive») **retry**-er den etter `reject_retry_sec`. Bruk **2D Pose Estimate** i RViz når sim er oppe — du trenger ikke å «rekke» initial pose innen en kort `start_delay` før Nav2 er aktivert.

Før mål sendes valideres alle waypoints mot **trygg indre boks**: kart-AABB fra **`/map`** (evt. `map_yaml_path` fra launch) minus **`map_margin_m`** (standard **0,75 m**). Punkter utenfor logges som **ERROR** og misjonen sendes **ikke**. Ved **ABORT** logges status, `error_code` / `error_msg`, siste TF-pose mot trygg boks og siste mottatte **global costmap**-AABB (hvis noen melding er mottatt). Nav2-feedback logges **høyst 1 Hz**, og bare når **`poses_remaining` endres** eller **`distance_remaining` endrer seg minst 0,25 m**.

### Kort stabilitetstest (anbefalt først)

```bash
ros2 launch moonmapper_nav2 nav2_short_mission_test.launch.py rviz:=true
```

Bruk **`nav2_params_mission_fast.yaml`** (høyere `vx_max` / `velocity_smoother`) og **`nav2_short_waypoints.yaml`** (få punkter innenfor trygg sone). Full coverage-test: `nav2_coverage_mission_test.launch.py` (standard fortsatt `nav2_params_moonmapper.yaml` — overstyr med `params_file:=.../nav2_params_mission_fast.yaml` om du vil samme hastighet der).

## Odom-only Nav2 test (uten AMCL, map_server eller 2D Pose Estimate)

Isolerer **planner/controller/costmap** i **`odom`**. Bringup kjøres med **`use_localization:=False`** (ingen AMCL, ingen map_server). Mål og costmaps bruker **`odom`** som global ramme.

### Nav2 odom baseline (første stabile test med Nav2 Goal i RViz)

Dette er den **rene** Nav2-baselinen etter at drivlinje/fysikk er verifisert: **ingen** `map_server`, **ingen** AMCL, **ingen** statisk kart og **ingen** 2D Pose Estimate. Global og lokal costmap er **rolling** i **`odom`** (`config/nav2_params_odom_baseline.yaml`). LaserScan til costmap: **`/scan`** (fra `depth_to_scan_node`, samme topic som øvrige odom-launch). Controller: **Regulated Pure Pursuit**; planner: **NavFn**; `velocity_smoother` er satt som i baseline-spesifikasjonen.

**Terminal 1 — sim:**

```bash
source install/setup.bash
ros2 launch moonmapper_bringup sim_rover_clean.launch.py
```

**Terminal 2 — Nav2 odom baseline:**

```bash
source install/setup.bash
ros2 launch moonmapper_nav2 nav2_odom_baseline.launch.py
```

(Standard: `rviz:=true` og `use_sim_time:=true`.) RViz: `rviz/moonmapper_nav2_odom_baseline.rviz` (**Fixed Frame: odom**). Kart- og AMCL-visning er av for å unngå støy.

**Terminal 3 — valgfri TF-sjekk:**

```bash
source install/setup.bash
ros2 run tf2_ros tf2_echo odom base_footprint
```

**Manuell test:** Bruk **ikke** 2D Pose Estimate. I RViz, bekreft **Fixed Frame: odom**. Bruk **Nav2 Goal** rett foran roboten (ca. 0,5–1,0 m), deretter et mål til siden (fortsatt nært), til slutt 2–3 m.

**Akseptanse (odom baseline):** Nav2 blir **active** uten AMCL; ingen gjentatte advarsler om manglende **`map`→`base_footprint`**; ingen «robot out of bounds of the costmap» / «Start Coordinates outside bounds» / `worldToMap failed`; global og lokal costmap **følger** roboten; roboten beveger seg mot valgt **Nav2 Goal**; **`/cmd_vel`** har fornuftige verdier; **`/odom`** (yaw og twist) stemmer overens med bevegelse i Gazebo.

**Terminal 2 — odom-misjon (waypoints, egen launch):**

```bash
source install/setup.bash
ros2 launch moonmapper_nav2 nav2_odom_short_mission.launch.py rviz:=true
```

Forvent: ingen «AMCL cannot publish…», ingen **2D Pose Estimate**, ingen TF-timeout mot `map`. RViz bruker **Fixed Frame: odom** (`rviz/moonmapper_nav2_odom_mission.rviz`). Misjonsnoden heter **`nav2_odom_mission_client`**; manuell start:  
`ros2 service call /nav2_odom_mission_client/start_mission std_srvs/srv/Trigger`.

Konfig for misjons-launch: `config/nav2_params_odom_test.yaml`, waypoints: `config/nav2_odom_short_waypoints.yaml`.

### cmd_vel / odom-retning (før du stoler på map+AMCL)

1. **Fremover:**

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.10}, angular: {z: 0.0}}"
```

Forvent: robot fremover i Gazebo; odom `twist.twist.linear.x` følger samme fortegn; yaw stabilt.

2. **Rotasjon (positiv angular z = CCW i plan):**

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.4}}"
```

Forvent: CCW-rotasjon; yaw endres konsistent.

Stemmer ikke dette: sjekk cmd_vel-relay, linear/angular-sign, og `diff_drive_controller` hjulnavn — **ikke** bruk AMCL før dette er verifisert.

### Verifiser at NavigateThroughPoses brukes

```bash
ros2 action list | grep -i navigate
ros2 action info /navigate_through_poses
```

I terminalen til `nav2_mission_client_node` skal du se logger om kart-/safe bounds, «sending NavigateThroughPoses goal», throttlet **Nav2 feedback**, og til slutt **SUCCEEDED** / avbrudd med detaljert diagnose ved **ABORT**.

I RViz (mission-visning): **Navigation 2**-panelet, **Global Plan** / **Local Plan**, partikler for AMCL, og at roboten følger en plan gjennom flere mål.

### Akseptansetest (kort)

1. **Terminal 1 — Gazebo:** `ros2 launch moonmapper_bringup sim_rover_clean.launch.py`
2. **Terminal 2 — kort misjon (anbefalt):** `ros2 launch moonmapper_nav2 nav2_short_mission_test.launch.py rviz:=true`  
   Eller full coverage: `ros2 launch moonmapper_nav2 nav2_coverage_mission_test.launch.py rviz:=true`  
   Eller **odom-only (uten AMCL):** `ros2 launch moonmapper_nav2 nav2_odom_short_mission.launch.py rviz:=true`
3. I RViz: **2D Pose Estimate** på roveren (**ikke** nødvendig for odom-testen). Launch logger **faktisk `map_file` / `params_file` / `waypoints_file`** ved oppstart (map-baserte launch).
4. Forvent: ingen «Start Coordinates outside bounds» / «Robot is out of bounds» pga. waypoints utenfor margin; `/cmd_vel` opp mot **~0,15–0,20 m/s** på rette strekninger med `mission_fast`; misjon **SUCCEEDED**. For odom-test: `distance_remaining` synker over tid; ingen `map`-frame i Nav2.

### Vanlige feil (Nav2-misjon)

| Symptom | Mulig årsak |
|--------|----------------|
| Waypoint avvist før send / ERROR safe bounds | Juster `waypoints` eller øk `map_margin_m`; ikke legg mål nærmere enn margin fra kartkant. |
| «Start outside bounds» / sensor utenfor kart | Bruk padded `map_file`; hold rute i indre boks; sjekk at `global_costmap.rolling_window` er **false** (standard i `nav2_params_moonmapper.yaml`). |
| «Robot out of costmap» / ingen plan | Initial pose mangler eller feil; TF `map`→`base_footprint` mangler. |
| «Action server inactive» / avvist mål | `bt_navigator` ikke **active** (lifecycle); eller Nav2-konflikt med ødelagt costmap-plugin overlay (se baseline README). |
| `planner_server` krasjer (f.eks. exit -11) | Gammel/ugyldig kart eller korrupt install av `nav2_costmap_2d` — restart launch, bytt `map_file`, se feilsøking i baseline-seksjonen under. |
| AMCL «feil» pose | Manglende **2D Pose Estimate** etter oppstart. |
| Ingen `/scan` | `depth_to_scan_node` kjører ikke eller kamera/bridge feiler — Nav2 costmap trenger scan i denne testen. |
| Robot beveger seg ikke / `/cmd_vel` null | Nav2 sender til `/cmd_vel_raw`; sjekk at `safety_obstacle_node` ikke blokkerer (timeout/scan), og at ingen annen node overskriver `/cmd_vel`. |

## Baseline v2 (padded kart + høyere hastighet)

- **Padded kart** unngår `worldToMap` / «robot out of bounds» når rover/sensor går litt utenfor det opprinnelige 97×47-kartet.
- **Hastighet:** MPPI `vx_max` 0.18 m/s, `wz_max` 0.68 rad/s; `velocity_smoother.max_velocity` matcher; lokal costmap 6×6 m; global inflasjon 0.35 m; NavFn `tolerance` 1.0 celle for kantmål.
- **Gazebo:** `diff_drive_controller` lineær max 0.30 m/s (headroom over Nav2).

### Generer padded kart på nytt

```bash
cd ~/rover/simulasjon/moonmapper_ws
source install/setup.bash
ros2 run moonmapper_mapping pad_occupancy_map.py --help
ros2 run moonmapper_mapping pad_occupancy_map.py \
  --input-yaml "$(ros2 pkg prefix moonmapper_mapping)/share/moonmapper_mapping/maps/moonmapper_test_map.yaml" \
  --output-dir "$(ros2 pkg prefix moonmapper_mapping)/share/moonmapper_mapping/maps" \
  --output-stem moonmapper_test_map_padded \
  --padding-m 2.0
```

(Etter `colcon build` ligger scriptet som `ros2 run moonmapper_mapping pad_occupancy_map.py`; under utvikling kan du også kjøre `python3 src/moonmapper_mapping/scripts/pad_occupancy_map.py ...`.)

### Health check (mens sim + Nav2 kjører)

```bash
source install/setup.bash
ros2 run moonmapper_nav2 nav2_health_check.sh
```

### Health check (Nav2 mission-stack)

```bash
source install/setup.bash
ros2 run moonmapper_nav2 nav2_mission_health_check.sh
```

## Filer

| Fil | Formål |
|-----|--------|
| `config/nav2_params_moonmapper.yaml` | Nav2-parametre (MPPI, costmaps, `cmd_vel_raw`, hastighet v2). |
| `config/nav2_params_mission_fast.yaml` | Raskere testprofil: MPPI `vx_max` 0,20, `vx_min` 0,05, `wz_max` 0,60 + `velocity_smoother` justert. |
| `config/nav2_coverage_waypoints.yaml` | Lengre (men fortsatt margin-sikret) flat waypoint-liste. |
| `config/nav2_short_waypoints.yaml` | Kort rute (5 poses) for stabilitetstest. |
| `moonmapper_nav2/nav2_mission_client_node.py` | Action-klient: `NavigateThroughPoses`. |
| `scripts/nav2_mission_client_node` | Tynn kjørbar (kaller `main()`). |
| `launch/nav2_localization_test.launch.py` | `localization_launch.py` (map_server + AMCL) + valgfri RViz. |
| `launch/nav2_static_map.launch.py` | **Anbefalt demo:** statisk kart + `localization_mode:=odom` (standard) eller `amcl`. |
| `launch/nav2_odom_static_map.launch.py` | Odom-modus: identity `map`→`odom`, `map_server`, ingen AMCL, ingen 2D Pose Estimate. |
| `launch/nav2_amcl_static_map.launch.py` | AMCL-modus: auto `initial_x/y/yaw` (match Gazebo `spawn_x/y`). |
| `launch/nav2_static_map_mission_test.launch.py` | `nav2_static_map` + kort `NavigateThroughPoses`-misjon. |
| `config/nav2_params_odom_static_map.yaml` | Nav2-parametre for odom static-map. |
| `config/nav2_params_amcl_static_map.yaml` | Som over + AMCL `set_initial_pose`. |
| `scripts/nav2_localization_diagnose.sh` | Lifecycle, TF, costmap, cmd_vel (bruk `odom` eller `amcl` som arg). |
| `launch/nav2_bringup_test.launch.py` | Eldre test: AMCL + full bringup (krever ofte 2D Pose Estimate). |
| `launch/nav2_coverage_mission_test.launch.py` | Baseline bringup + misjonsnode; logger **resolved** `map_file` / `params_file` / `waypoints_file`. |
| `launch/nav2_short_mission_test.launch.py` | Kort test: **mission_fast** + **short** waypoints som standard. |
| `rviz/moonmapper_nav2.rviz` | RViz (utgangspunkt: Jazzy `nav2_default_view.rviz`). |
| `rviz/moonmapper_nav2_mission.rviz` | RViz for misjonstest (RobotModel påslått m.m.). |
| `scripts/nav2_health_check.sh` | Sjekker `/map`, costmaps, lifecycle, `cmd_vel`-kjede og kart-AABB. |
| `scripts/nav2_mission_health_check.sh` | Actions, lifecycle, `/plan`, costmaps, TF, `cmd_vel`-kjede. |

Padded kart genereres med **`moonmapper_mapping`**-scriptet `pad_occupancy_map.py` (se mapping-README).

## Bygg

```bash
cd ~/rover/simulasjon/moonmapper_ws
colcon build --packages-select moonmapper_autonomy moonmapper_mapping moonmapper_nav2 --symlink-install
source install/setup.bash
```

## Test Nav2 (kun denne stacken)

### Statisk kart — odom-modus (standard demo, ingen 2D Pose Estimate)

**Terminal 1 — Gazebo**

```bash
ros2 launch moonmapper_bringup sim_rover_clean.launch.py
```

**Terminal 2 — Nav2 static map (identity map→odom)**

```bash
source install/setup.bash
ros2 launch moonmapper_nav2 nav2_static_map.launch.py
# eller eksplisitt:
ros2 launch moonmapper_nav2 nav2_odom_static_map.launch.py
```

RViz: Fixed Frame **map**, bruk **Nav2 Goal** (ikke 2D Pose Estimate). Diagnose:

```bash
ros2 run moonmapper_nav2 nav2_localization_diagnose.sh odom
```

**Automatisk kort misjon**

```bash
ros2 launch moonmapper_nav2 nav2_static_map_mission_test.launch.py
```

### Statisk kart — AMCL-modus (eksperimentell)

```bash
ros2 launch moonmapper_nav2 nav2_static_map.launch.py localization_mode:=amcl \
  initial_x:=0.0 initial_y:=0.0 initial_yaw:=0.0
ros2 run moonmapper_nav2 nav2_localization_diagnose.sh amcl
```

Match `initial_x/y/yaw` til Gazebo-spawn.

### Eldre baseline (AMCL + manuell pose)

**Terminal 1 — Gazebo** (som over)

**Terminal 2 — Nav2 + safety (baseline)**

```bash
source install/setup.bash
ros2 launch moonmapper_nav2 nav2_bringup_test.launch.py rviz:=true
```

**Terminal 2 — Nav2 + safety + automatisk NavigateThroughPoses-misjon (coverage, standard hastighet)**

```bash
source install/setup.bash
ros2 launch moonmapper_nav2 nav2_coverage_mission_test.launch.py rviz:=true
```

**Samme med rask profil**

```bash
ros2 launch moonmapper_nav2 nav2_coverage_mission_test.launch.py rviz:=true \
  params_file:=$(ros2 pkg prefix moonmapper_nav2)/share/moonmapper_nav2/config/nav2_params_mission_fast.yaml
```

**Kort misjon (standard mission_fast)**

```bash
ros2 launch moonmapper_nav2 nav2_short_mission_test.launch.py rviz:=true
```

Overstyr kart, Nav2-params og waypoint-fil:

```bash
ros2 launch moonmapper_nav2 nav2_coverage_mission_test.launch.py \
  rviz:=true \
  map_file:=$(ros2 pkg prefix moonmapper_mapping)/share/moonmapper_mapping/maps/moonmapper_test_map_padded.yaml \
  params_file:=$(ros2 pkg prefix moonmapper_nav2)/share/moonmapper_nav2/config/nav2_params_moonmapper.yaml \
  waypoints_file:=$(ros2 pkg prefix moonmapper_nav2)/share/moonmapper_nav2/config/nav2_coverage_waypoints.yaml
```

(Standard `map_file` er padded kart under `moonmapper_mapping/maps/`. Eksplisitt: `map_file:=$(ros2 pkg prefix moonmapper_mapping)/share/moonmapper_mapping/maps/moonmapper_test_map_padded.yaml`.)

I RViz: **2D Pose Estimate** for AMCL, deretter **Navigate To Pose** (eller 2D Goal avhengig av visning). Forvent bevegelse mot mål (tuning kan forbedres).

**Rekkefølge:** Sim må kjøre først (så `odom`→`base_footprint` finnes). Deretter Nav2-launch. **Før** du forventer robotmodell i **Fixed Frame = map**: bruk **2D Pose Estimate** (AMCL publiserer da `map`→`odom`). Uten det viser RViz «No transform from … to map» for alle lenker.

**Verifiser TF:**

```bash
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo map odom
```

Første skal gi data når Gazebo kjører. Andre gir data først etter vellykket initial pose (eller kortvarig under AMCL-oppstart).

## Feilsøking

### 1) `costmap_plugins.xml` / «has no Root Element» / `VoxelLayer` finnes ikke

Da lastes **ikke** costmap-plugins. Ofte skyldes det at `moonmapper_ws/install/nav2_costmap_2d/...` er en **symlink** til en **mangelfull eller slettet** mappe (f.eks. `ros-jazzy-nav2-costmap-2d-...` i workspace), slik at XML-filen er tom eller ugyldig. `AMENT_PREFIX_PATH` prioriterer workspace foran `/opt/ros/jazzy`, og pluginlib feiler.

**Sjekk:**

```bash
readlink -f ~/rover/simulasjon/moonmapper_ws/install/nav2_costmap_2d/share/nav2_costmap_2d/costmap_plugins.xml
wc -c ~/rover/simulasjon/moonmapper_ws/install/nav2_costmap_2d/share/nav2_costmap_2d/costmap_plugins.xml
head -3 /opt/ros/jazzy/share/nav2_costmap_2d/costmap_plugins.xml
```

Workspace-filen skal ha samme størrelse/innhold som under `/opt/ros/jazzy/`.

**Rett opp:** fjern ødelagt overlay og bygg kun egne pakker (eller bruk apt-nav2 på nytt):

```bash
rm -rf ~/rover/simulasjon/moonmapper_ws/install/nav2_costmap_2d \
       ~/rover/simulasjon/moonmapper_ws/build/nav2_costmap_2d
# Hvis du hadde en halvferdig kildekopi som symlinken pekte på:
rm -rf ~/rover/simulasjon/moonmapper_ws/ros-jazzy-nav2-costmap-2d-*
sudo apt install --reinstall ros-jazzy-nav2-costmap-2d
source /opt/ros/jazzy/setup.bash
cd ~/rover/simulasjon/moonmapper_ws && source install/setup.bash
ros2 pkg prefix nav2_costmap_2d
```

`ros2 pkg prefix nav2_costmap_2d` skal peke til **`/opt/ros/jazzy/...`** hvis du ikke bygger nav2 fra kilde i dette workspace.

Start deretter `nav2_bringup_test` på nytt. Da skal **Navigation** kunne bli **active** (ikke «inactive»), og Navigate To Pose skal ikke avvises med «Action server is inactive».

### 2) RobotModel: «No transform from … to map»

Med **Fixed Frame = map** kreves kjeden `map` → `odom` → `base_footprint` → … AMCL publiserer **`map`→`odom` først etter initial pose**. Gazebo/robot gir **`odom`→`base_footprint`**.

- Løsning: **2D Pose Estimate** i RViz (klikk omtrent der roveren står i sim).
- Hurtigtest uten kart: sett midlertidig **Fixed Frame** til **`odom`** i RViz — da skal modellen ofte vises hvis sim kjører, men kartet stemmer ikke med robot før du går tilbake til `map` og setter pose.

### 3) Navigate To Pose: «aborted» / `Goal ... was outside bounds`

Planneren (NavFn) avviser mål som ligger **utenfor global costmap-rutenettet** — for statisk kart tilsvarer det stort sett **kartets .yaml/.pgm-rektangel i `map`-frame** (ikke bare den hvite sonen i RViz).

For `moonmapper_test_map.yaml` (97×47 celler, oppløsning 0.05 m, `origin: [-3.992, -2.285, 0]`):

- **x** fra ca **−3.99** til **+0.86** m  
- **y** fra ca **−2.29** til **+0.07** m  

Eksempel fra logg: mål **y ≈ 0.09** er **over** nordgrensen (~0.065 m) → `outside bounds`. Velg mål **innenfor** dette rektangelet (gjerne litt inn fra kanten så inflasjon ikke markerer målcellen som uløselig).

Konfigurasjonen i `nav2_params_moonmapper.yaml` har **`global_costmap.rolling_window: false`** slik at hele lagret kart brukes som planleggingsgrid. Med **`moonmapper_test_map_padded.yaml`** (177×127 celle, origin ca **−5.99, −4.29**) får du ~**2 m** ekstra fri plass på alle sider uten å flytte det opprinnelige kartinnholdet i verden.

## Debug

```bash
ros2 node list | grep -E 'nav2|amcl|map_server|controller'
ros2 lifecycle nodes
ros2 topic echo /plan --once
ros2 topic echo /cmd_vel_raw --once
ros2 topic echo /cmd_vel --once
```

## Se også

Hoveddokumentasjon for hastighetsargumenter og custom-stack: `moonmapper_mapping/README.md` (seksjon om hastighet + Nav2).
