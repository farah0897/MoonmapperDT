# Nav2 static map — Steg 4: replan rundt hindring

Validerer at roboten med **localization_mode=odom** (identity `map`→`odom`, ingen AMCL) kan oppdage et dynamisk hinder, oppdatere costmaps/plan, styre rundt og nå mål bak hindringen.

## Arkitektur (uendret)

- `nav2_static_map.launch.py` (`localization_mode:=odom`)
- Padded kart `moonmapper_test_map_padded.yaml`
- Ingen 2D Pose Estimate
- `/scan` fra `depth_to_scan_node`
- `safety_obstacle_node`: stopper kun **fremover** (`linear.x`) i smal front-sektor; **beholder `angular.z`**

## Oppsett

| Element | Standard |
|---------|----------|
| Robot spawn | `(0, 0)` |
| Hindring (Gazebo-blokk) | `(0.75, 0.0)` — `replan_test_block` |
| Mål (map) | `(2.0, 0.0)` bak hindringen |
| Fri sidepass | ±y ≈ 0.5–1.0 m rundt blokk (0.55 m bred) |

## Kjøring

**Terminal 1**

```bash
ros2 launch moonmapper_bringup sim_rover_clean.launch.py
```

**Terminal 2**

```bash
source install/setup.bash
ros2 launch moonmapper_nav2 nav2_static_map_replan_test.launch.py
```

Med manuelt mål i RViz (ingen auto-goal):

```bash
ros2 launch moonmapper_nav2 nav2_static_map_replan_test.launch.py autostart_goal:=false
```

Plasser **Nav2 Goal** på f.eks. `(2.0, 0.0)` eller `(2.0, -0.6)` (gå rundt på høyre side).

## Hva skal verifiseres

| # | Sjekk | Hvor |
|---|--------|------|
| 1 | `/scan` ser hindring foran | `replan:` log `scan_front_min` < ~1.0 m |
| 2 | Lokal costmap markerer hinder | RViz `/local_costmap/costmap`, log `local_lethal` > 0 |
| 3 | Global costmap (obstacle_layer) | RViz `/global_costmap/costmap`, log `global_lethal` > 0 |
| 4 | `/plan` endres ved blokkert rett vei | log `plan_changes` øker, `plan_poses` endres |
| 5 | Controller styrer | log `raw_v` har `angular.z` ≠ 0 under omkjøring |
| 6 | Safety blokkerer ikke sving | `safety=clear` eller `blocked_front` med `safe_ang` ≈ `raw_ang` |
| 7 | Innenfor kart | `in_map_margin=true` |
| 8 | Resultat | `nav=done status=4` (succeeded) eller kontrollert abort |

## Parametre (referanse)

Fil: `config/nav2_params_odom_static_map.yaml`

**Global costmap `obstacle_layer`:** `marking: true`, `clearing: true`, `obstacle_max_range` 2.5 m, `raytrace_max_range` 3.0 m.

**Local `voxel_layer`:** samme scan-kilde, `inflation_radius` 0.32 m.

**Controller (MPPI):** `vx_max` 0.30, `wz_max` 0.90.

**Safety (replan-launch):** `safety_stop_distance` 0.48 m, `safety_front_angle_deg` 38° — smal front-cone slik at sidepass med rotasjon tillates.

## Logging (`nav2_replan_monitor`)

~1 Hz på stdout, f.eks.:

```
replan: pose=(0.42,0.08,yaw=12.3°) goal=(2.00,0.00) scan_front_min=0.62 safety=clear
  raw_v=(0.12,0.45) safe_v=(0.12,0.45) plan_poses=48 plan_changes=2 local_lethal=12 global_lethal=8 nav=active
```

## Suksess

- Robot kjører **rundt** blokken (ikke inn i den)
- `bt_navigator` / monitor: **Goal succeeded**
- Ingen «out of bounds» / krasj

## Feilsøking

| Symptom | Tiltak |
|---------|--------|
| Ingen blokk i `/scan` | Vent 3 s etter launch (spawn timer); sjekk at sim kjører |
| `plan_changes=0` | Vent til `bt_navigator` active; sjekk at mål er bak hindring |
| Safety stopper alltid | Senk ikke `angular.z`; sjekk `safety_front_angle_deg` (smalere = bedre sidepass) |
| Goal aborted | Prøv mål `(2.0, -0.7)` med tydelig sidepass |

## Ikke i scope

SLAM, ukjent terreng, AMCL, nye kart.
