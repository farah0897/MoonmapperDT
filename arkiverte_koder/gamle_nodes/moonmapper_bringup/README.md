# Arkiverte `moonmapper_bringup`-noder

**Dato:** 2026-05-17

## Aktivt (fortsatt i `src/moonmapper_bringup/`)

| Fil | Rolle |
|-----|--------|
| `moonmapper_bringup/cmd_vel_odom_relay.py` | `/cmd_vel` ↔ diff_drive (startes fra `gazebo_rover`) |
| `launch/sim_rover_clean.launch.py` | Entry for `autonomous_exploration_full` |
| `config/moonmapper_rviz.rviz` | RViz ved `use_rviz:=true` |

## Arkiverte noder

| Fil | Tidligere bruk |
|-----|----------------|
| `camera_aliases.py` | `/depth_camera/*` → `/camera/camera/*` (fysisk Realense-konvensjon). Full autonom sim bruker `/depth_camera` direkte. |

Gjenopprett: flytt tilbake + `setup.py` entry_point + ev. start fra launch.
