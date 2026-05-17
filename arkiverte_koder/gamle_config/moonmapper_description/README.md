# Arkiverte configs — moonmapper_description

Flyttet 2026-05-17. Aktiv autonom stack (`sim_rover_clean` → `gazebo_rover`) bruker **ikke** EKF (`use_ekf:=false` default) og **ikke** joint_state_publisher_gui.

| Fil | Grunn |
|-----|--------|
| `jsp_drivers_only.yaml` | Kun for manuell `joint_state_publisher_gui` |
| `ekf_wheel_odom.yaml` | Kun når `gazebo_rover.launch.py use_ekf:=true` |
| `diff_drive_controller_disable_odom_tf.yaml` | Overlay sammen med EKF |

**Aktive configs** (fortsatt i `src/moonmapper_description/config/`): `ros_gz_bridge.yaml`, `wheel_controllers.yaml`, `diff_drive_controller.yaml`, `joint_state_broadcaster.yaml`, `physics_profiles.yaml`.

Gjenoppretting: flytt tilbake til `src/moonmapper_description/config/` før `use_ekf:=true`.
