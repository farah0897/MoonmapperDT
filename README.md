# MoonMapper (ROS 2)

MoonMapper er ryddet tilbake til en **basis-simulering**:

- Robotmodell (URDF/Xacro, mesh)
- Gazebo Sim
- `ros2_control` med `diff_drive_controller` og `joint_state_broadcaster`
- Sensorbroer (kamera/IMU m.m.) via `ros_gz_bridge`
- Enkel manuell kjøring (teleop eller direkte `cmd_vel`)

**Navigasjon (Nav2), UWB, EKF-eksperiment og tilhørende debug-verktøy er fjernet** og skal reimplementeres senere fra bunnen av.

## Anbefalt start (Gazebo)

Bygg og source:

```bash
cd /path/to/moonmapper_ws
colcon build --symlink-install
source install/setup.bash
```

Start ren sim (ingen Nav2, ingen EKF/UWB):

```bash
ros2 launch moonmapper_bringup sim_rover_clean.launch.py use_sim_time:=true
```

### Validering

- **Topics:** `ros2 topic list` — forvent bl.a. `/joint_states`, `/diff_drive_controller/odom`, `/clock`.
- **Kontrollere:** `ros2 control list_controllers` og `ros2 control list_hardware_interfaces`.
- **Direkte kjøring (Jazzy `TwistStamped`):**

```bash
ros2 topic pub --once /diff_drive_controller/cmd_vel geometry_msgs/msg/TwistStamped "{header: {frame_id: base_link}, twist: {linear: {x: 0.08}, angular: {z: 0.0}}}"
```

- **Odom:** `ros2 topic echo /diff_drive_controller/odom --once`

Alternativt teleop (se docstring i `gazebo_rover.launch.py` for `teleop_twist_keyboard` med `stamped:=true`).

## Andre launch-filer

- `moonmapper_description/launch/gazebo_rover.launch.py` — full Gazebo-oppsett (valgfritt `use_ekf:=true` bruker `moonmapper_description/config/ekf_wheel_odom.yaml` for hjul-odom EKF).
- `moonmapper_bringup/launch/sensor_bringup.launch.py` — tynn wrapper mot `gazebo_rover` (standard verden + RViz).
- `moonmapper_bringup/launch/sim.launch.py` — **Webots**-sim med valgfri EKF/SLAM (ikke Gazebo).

## Mer dokumentasjon

Se `Kodedokumentasjone.md` og `docs/` for øvrig prosjektbeskrivelse.
