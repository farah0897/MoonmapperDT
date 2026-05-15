# Nav2 static map odom baseline test

Repeterbar manuell test for den stabile bachelor-demo-stacken:

- `localization_mode:=odom` (standard)
- Statisk kart + identity `map`→`odom`
- Ingen AMCL, ingen RViz **2D Pose Estimate**
- Nav2 **Navigate To Pose** i RViz (Fixed Frame `map`)

Parametre: `config/nav2_params_odom_static_map.yaml` (MPPI `vx_max` 0.30, goal checker `xy_goal_tolerance` 0.10).

## Forutsetninger

```bash
cd ~/rover/simulasjon/moonmapper_ws
colcon build --packages-select moonmapper_nav2 --symlink-install
source install/setup.bash
```

## Prosedyre

### 1. Start sim (terminal 1)

```bash
ros2 launch moonmapper_bringup sim_rover_clean.launch.py
```

Vent til Gazebo og `/odom` publiseres.

### 2. Start Nav2 static map odom (terminal 2)

```bash
source install/setup.bash
ros2 launch moonmapper_nav2 nav2_static_map.launch.py
```

Standard er `localization_mode:=odom`. Ikke bruk **2D Pose Estimate**.

Valgfri diagnose (mens stacken kjører, terminal 3):

```bash
ros2 run moonmapper_nav2 nav2_localization_diagnose.sh odom
```

Forvent bl.a.:

- `map_server` lifecycle **active**
- Ingen `amcl`-node
- TF `map`→`odom` ≈ identity (0, 0, 0)
- TF `odom`→`base_footprint` oppdateres når roboten beveger seg
- `bt_navigator` **active** etter noen sekunder

### 3. RViz — send mål

1. Fixed Frame: **map**
2. Bruk **Nav2 Goal** (Navigate To Pose), ikke 2D Pose Estimate
3. Velg ett av de anbefalte målene under (innenfor padded testkart)

### 4. Suksesskriterier

| Sjekk | Forventet |
|-------|-----------|
| Planlegging | `/plan` vises i RViz under kjøring |
| Bevegelse | `safety_obstacle_node`: `state=clear`, `raw_lin` > 0 |
| Hastighet | `raw_lin` typisk opp mot ~0.30 m/s (cap i params) |
| Mål | `bt_navigator`: **Goal succeeded** |
| Posisjon | Robot innen ~0.10 m (xy) og ~0.25 rad (yaw) av mål |

Eksempel fra godkjent test: spawn ~`(0, 0)` → mål `x≈0.99`, robot nådde ~`x=0.838` underveis og fullførte med **Goal succeeded**.

## Anbefalte trygge testmål (map frame)

Innenfor `moonmapper_test_map_padded` (margin fra vegg). Yaw kan settes til `0` eller peke mot neste punkt.

| # | x (m) | y (m) | Merknad |
|---|-------|-------|---------|
| A | 1.5 | 0.0 | Fremover i korridor |
| B | 1.5 | -0.8 | Litt til høyre (neg. y) |
| C | 0.2 | -1.2 | Bakover/side, kort rute |

Test ett mål om gangen. Ved avvisning: sjekk at `planner_server` og `bt_navigator` er **active** (`ros2 lifecycle get /bt_navigator`).

## Kjente shutdown-advarsler (ikke Nav2-feil)

Ved **Ctrl+C** i launch-terminalen kan du se:

- `nav2_tf_chain_logger`: ingen `rclpy.shutdown()`-feil (skal avslutte med exit 0)
- RViz: `GLSL link error` / prosess exit `-6` — kjent ved hard shutdown av RViz; påvirker ikke Nav2-testen mens systemet kjører

## Neste steg (ikke del av denne testen)

- Lengre ruter: `nav2_static_map_mission_test.launch.py`
- AMCL-modus: `localization_mode:=amcl` (eksperimentell)
- Ren odom-rulling uten kart: `nav2_odom_baseline.launch.py`
