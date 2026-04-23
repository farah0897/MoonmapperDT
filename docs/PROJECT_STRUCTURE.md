## MoonMapper workspace – structure and “what is canonical”

This workspace contains **multiple simulation/integration stacks**:

- **Webots (primary, “MVP”)**
  - Entry point: `ros2 launch moonmapper_bringup sim.launch.py`
  - Package: `moonmapper_webots` (world + driver)
  - Robot model: `moonmapper_description/urdf/moonmapper_rover.urdf.xacro`

- **Gazebo Sim (Harmonic / gz-sim8)**
  - Entry point: `ros2 launch moonmapper_bringup sensor_bringup.launch.py`
  - Wrapper launch: `moonmapper_description/launch/gazebo_rover.launch.py`
  - Gazebo wrapper xacro: `moonmapper_description/urdf/moonmapper_rover_gazebo.urdf.xacro`
  - Bridges: `moonmapper_description/config/ros_gz_bridge.yaml`
  - Controllers: `moonmapper_description/config/wheel_controllers.yaml`
  - Custom plugins:
    - `moonmapper_description/gz_plugins` (rocker-bogie differential)
    - `moonmapper_gz_sensors/gz_plugins` (Level 1 triad spectroscopy)

- **Unity (optional / legacy integration path)**
  - ROS-TCP endpoint: `ros_tcp_endpoint` (vendored package)
  - Unity scripts live in `moonmapper_description/unity_scripts/`
  - NOTE: switching Gazebo <-> Unity should **not** rely on moving source files.

### Canonical assets

- **Robot URDF/Xacro**: `src/moonmapper_description/urdf/`
- **Robot meshes used at runtime**: `src/moonmapper_description/meshes/`
- **World assets**
  - Webots: `src/moonmapper_webots/worlds/`
  - Gazebo: `src/moonmapper_description/worlds/`

### Non-canonical / generated artifacts

These should be treated as exports/artifacts, not as the canonical source-of-truth for the robot model:

- `meshes/` (workspace root): raw CAD/STL exports (may duplicate `moonmapper_description/meshes`)
- `unity_export/`: URDF exported for Unity import pipelines
- `frames_*.{pdf,gv}`: generated TF diagrams
- `build/`, `install/`, `log/`: colcon artifacts

