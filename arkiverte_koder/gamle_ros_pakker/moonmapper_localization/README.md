# moonmapper_localization (arkivert)

**Arkivert:** 2026-05-17

Legacy ROS 2-pakke med kun `config/ekf.yaml` for `robot_localization` / `ekf_filter_node` (fusjon av `/odom`, valgfritt IMU).

## Ikke brukt i aktiv stack

Autonom Gazebo-sim bruker **RTAB-Map** for kart/TF og hjul-odometri via `/odom` (relay fra `diff_drive_controller`). `use_ekf:=false` i `sim_rover_clean`.

Gazebo-spesifikk EKF lå i `moonmapper_description/config/ekf_wheel_odom.yaml` (også arkivert under `gamle_config/moonmapper_description/`).

## Gjenoppretting

```bash
git mv arkiverte_koder/gamle_ros_pakker/moonmapper_localization src/moonmapper_localization
colcon build --packages-select moonmapper_localization
```

Webots `gamle_webots_filer/sim.launch.py` forventet denne pakken i `src/`.
