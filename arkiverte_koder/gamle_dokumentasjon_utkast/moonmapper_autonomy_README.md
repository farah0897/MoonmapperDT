# moonmapper_autonomy

## Steg 2b: Depth-kamera → `/scan` + reaktiv + safety (anbefalt)

`/scan` genereres fra **RealSense/depth-kamera** (`depth_to_scan_node`), ikke fra Gazebo-LiDAR.

**Kjede:** `depth_camera` → `/scan` → `reactive_avoidance_node` → `/cmd_vel_raw` → `safety_obstacle_node` → `/cmd_vel` → `cmd_vel_odom_relay` → diff_drive.

**Terminal 1 — sim:**

```bash
cd ~/rover/simulasjon/moonmapper_ws
source install/setup.bash
ros2 launch moonmapper_bringup sim_rover_clean.launch.py
```

**Terminal 2 — depth + reactive + safety:**

```bash
cd ~/rover/simulasjon/moonmapper_ws
source install/setup.bash
ros2 launch moonmapper_autonomy depth_reactive_avoidance.launch.py
```

**Terminal 3 — observasjon:**

```bash
source ~/rover/simulasjon/moonmapper_ws/install/setup.bash
ros2 topic echo /scan --once
ros2 topic hz /scan
ros2 topic echo /cmd_vel_raw
ros2 topic echo /cmd_vel
ros2 topic echo /obstacle/front_min
ros2 topic echo /obstacle/current_state
```

### Godkjenning

- Ingen ekstra sylinder/boks for LiDAR på toppen av roveren i Gazebo.
- `/scan` bygges fra depth med **vertikal ROI** (lavere del av bildet) for å se lave hindringer tidligere.
- **Debug:** `/obstacle/front_min`, `/obstacle/left_min`, `/obstacle/right_min` (Float32, `nan` = ingen treff), `/obstacle/current_state` (String ved tilstandsskifte).
- Reaktiv: **FORWARD** med **slow zone** (0,02 m/s mellom 0,80–1,20 m), full fart **0,04 m/s** over 1,20 m; **AVOID** med **±0,65 rad/s**; hysterese **0,80 / 1,00 m**, median 3, **min. 1,2 s** i unnamanøver.
- Safety på `/cmd_vel`: stopp lineær ved **0,65 m**, **45°** sektor, `scan_timeout_sec` **0,5 s**.
- Ingen Nav2/SLAM/EKF/UWB fra disse launch-filene.

---

## Steg 3: Coverage-logging + validering (observerer kun)

`coverage_logger_node` bygger et **10×10 m** rutenett i **odom** med **0,10 m** celle, sentrert på **første** `/odom`-posisjon. Den publiserer dekningsprosent, besøkte celler, kjørt distanse og status (**RUNNING** / **STUCK_WARNING** / **FINISHED**). Robot styres fortsatt av depth + reactive + safety; denne noden **påvirker ikke** `cmd_vel`.

**Terminal 1 — sim:**

```bash
cd ~/rover/simulasjon/moonmapper_ws
source install/setup.bash
ros2 launch moonmapper_bringup sim_rover_clean.launch.py
```

**Terminal 2 — full stack + coverage:**

```bash
source install/setup.bash
ros2 launch moonmapper_autonomy autonomy_coverage_test.launch.py
```

**Terminal 3 — tall:**

```bash
source install/setup.bash
ros2 topic echo /coverage/percent
ros2 topic echo /coverage/distance_traveled
ros2 topic echo /coverage/status
ros2 topic echo /coverage/current_cell
```

**Terminal 4 — valgfritt:**

```bash
ros2 topic echo /obstacle/current_state
ros2 topic echo /cmd_vel
```

**Kun coverage (krever at `/odom` allerede finnes):**

```bash
ros2 launch moonmapper_autonomy coverage_logger.launch.py
```

**RViz:** Fixed frame **odom**, legg til **Map**, topic **`/coverage/grid`**. Verdier: **0** = ikke besøkt, **100** = besøkt.

### Godkjenning Steg 3

- `/coverage/grid`, `/coverage/percent`, `/coverage/visited_cells`, `/coverage/distance_traveled`, `/coverage/current_cell`, `/coverage/status` oppdateres mens sim kjører.
- `distance_traveled` og `visited_cells` øker når roboten kjører; `STUCK_WARNING` hvis posisjon nesten ikke endrer seg i `stuck_check_window_sec`; `FINISHED` etter `mission_duration_sec` (logg kun — ingen auto-stopp).
- Ingen Nav2/SLAM/EKF/UWB fra disse launch-filene.

---

## Kun depth + safety (uten reactive)

```bash
ros2 launch moonmapper_autonomy depth_scan_safety_test.launch.py
```

---

## Eldre: safety drive test (krever egen `/scan`-kilde)

Hvis du ikke starter `depth_to_scan_node`, må `/scan` komme fra annen kilde.

```bash
ros2 launch moonmapper_autonomy safety_drive_test.launch.py
```
