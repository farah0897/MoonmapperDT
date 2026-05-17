# Arkiverte `moonmapper_autonomy`-noder

**Dato:** 2026-05-17

Noder som **ikke** startes av `autonomous_exploration_full` / `nav2_rtabmap_navigation`.  
Test-launch ligger i `arkiverte_koder/gamle_sensor_tester/moonmapper_autonomy/`.

## Aktive noder (fortsatt i `src/moonmapper_autonomy/moonmapper_autonomy/`)

| Fil | Rolle |
|-----|--------|
| `depth_to_scan_node.py` | `/depth_camera/*` → `/scan` |
| `safety_obstacle_node.py` | `/cmd_vel_raw` → `/cmd_vel` |

## Arkiverte noder (denne mappen)

| Fil | Tidligere bruk |
|-----|----------------|
| `reactive_avoidance_node.py` | Reaktiv unnamanøvre uten Nav2 |
| `goal_obstacle_avoidance_node.py` | Goal + scan-detour |
| `simple_goal_follower_node.py` | Enkel `/goal_pose`-follower |
| `coverage_logger_node.py` | Dekningsrutenett (kun logging) |

Gjenoppretting: `git mv` tilbake til `src/moonmapper_autonomy/moonmapper_autonomy/` og legg til entry_points i `setup.py`.
