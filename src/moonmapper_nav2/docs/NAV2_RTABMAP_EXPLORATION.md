# Nav2 + RTAB-Map exploration (real robot)

Live SLAM navigation for the physical MoonMapper rover. **Does not modify** `nav2_static_map.launch.py` (sim static-map baseline).

## Architecture

```
RealSense D435 → RGB/depth/pointcloud
robot.py       → /odom + TF odom→base_link (EKF)
static TF      → base_link→camera_*
RTAB-Map       → /rtabmap/map + TF map→odom
topic_tools    → relay /rtabmap/map → /map
depth_to_scan  → /scan (Nav2 costmaps)
Nav2           → /cmd_vel_raw → safety_obstacle → /cmd_vel
robot.py       → motors
frontier (Ph3)→ goals to Nav2
```

| Component | NOT used |
|-----------|----------|
| map_server (static) | ✗ |
| AMCL | ✗ |
| 2D Pose Estimate | ✗ |
| identity `map`→`odom` static TF | ✗ |

| Frame | Owner |
|-------|--------|
| `odom`→`base_link` | `robot.py` |
| `map`→`odom` | RTAB-Map |
| `base_link`→`camera_*` | static TF launch |

Nav2 `robot_base_frame`: **`base_link`** (`nav2_params_rtabmap_real.yaml`).

## Dependencies (Jetson)

See `rover/setup/installation.md`:

- `ros-$ROS_DISTRO-rtabmap-ros` / `rtabmap_launch`
- `ros-$ROS_DISTRO-realsense2-camera`
- `ros-$ROS_DISTRO-topic-tools`
- `ros-$ROS_DISTRO-nav2-bringup`
- `frontier_exploration_ros2` (Phase 3 only)

```bash
cd ~/rover/simulasjon/moonmapper_ws
colcon build --packages-select moonmapper_bringup moonmapper_nav2 moonmapper_autonomy --symlink-install
source install/setup.bash
```

## Phased rollout

### Phase 1 — stack without spin or frontier

#### Simulation (PC + Gazebo)

**Terminal 1 — sim (MÅ kjøre først):**
```bash
source install/setup.bash
ros2 launch moonmapper_bringup sim_rover_clean.launch.py
```

**Terminal 2 — RTAB + Nav2 sim:**
```bash
source install/setup.bash
ros2 launch moonmapper_nav2 nav2_rtabmap_exploration_sim.launch.py nav2_rviz:=true
```

Preflight venter på `/depth_camera/*`, `/odom`, TF `odom`→`base_footprint` før RTAB starter.

#### Real robot

**Terminal A — motor + EKF (required):**
```bash
python3 ~/robot.py
```
Wait for `Robot ready!`

**Terminal B — navigation stack:**
```bash
source install/setup.bash
ros2 launch moonmapper_bringup real_robot_navigation.launch.py
# or:
ros2 launch moonmapper_nav2 nav2_rtabmap_exploration_real.launch.py
```

Optional RViz:
```bash
ros2 launch moonmapper_bringup real_robot_navigation.launch.py nav2_rviz:=true rtabmap_rviz:=true
```

**Acceptance:** Diagnose script passes; send **Nav2 Goal** in RViz (Fixed Frame `map`) after `/map` has data.

```bash
ros2 run moonmapper_nav2 nav2_rtabmap_diagnose.sh
```

### Phase 2 — initial spin + map ready

After Phase 1 works with manual goals:

```bash
ros2 launch moonmapper_bringup real_robot_navigation.launch.py enable_initial_spin:=true
```

Spins ~360° at `angular.z=0.3` after `/map` is non-empty (builds coverage before exploration).

### Phase 3 — frontier explorer

After manual Nav2 goals work reliably:

```bash
ros2 launch moonmapper_bringup real_robot_navigation.launch.py \
  enable_initial_spin:=true \
  enable_frontier_explorer:=true
```

Params: `config/frontier_params_moonmapper.yaml` (`map_topic=/map`, `robot_base_frame=base_link`, `strategy=mrtsp`).

## Manual diagnostic commands

```bash
ros2 topic echo /odom --once
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo map base_link
ros2 topic echo /rtabmap/map --once
ros2 topic echo /map --once
ros2 topic echo /cmd_vel --once
ros2 topic echo /scan --once
```

## Launch arguments (selected)

| Arg | Default | Description |
|-----|---------|-------------|
| `start_robot_driver` | `false` | Run `python3 ~/robot.py` from launch |
| `start_realsense` | `true` | D435 node + pointcloud enable |
| `start_rtabmap` | `true` | RTAB-Map SLAM |
| `start_map_relay` | `true` | `/rtabmap/map` → `/map` |
| `start_nav2` | `true` | Nav2 navigation stack |
| `enable_initial_spin` | `false` | Phase 2 |
| `enable_frontier_explorer` | `false` | Phase 3 |
| `camera_height_m` | `0.27` | `base_link`→`camera_link` z |

## Fresh RTAB-Map session

```bash
rm ~/.ros/rtabmap.db
```

## Sim note

Static-map sim baseline remains:

```bash
ros2 launch moonmapper_nav2 nav2_static_map.launch.py
```

RTAB exploration is intended for **real hardware**. For sim RTAB experiments later, use `sim_mode:=true` and Gazebo camera alias topics (not validated in CI).

## Acceptance checklist

1. No TF conflict: only RTAB publishes `map`→`odom` (no identity static TF from static-map launch)
2. `/odom` continuous
3. `odom`→`base_link` continuous
4. RTAB-Map builds map
5. `/rtabmap/map` relayed to `/map`
6. Nav2 lifecycle **active**
7. Manual RViz goal works on live map
8. Robot stays in costmap bounds
9. Frontier only after manual goals work (Phase 3)
10. `nav2_static_map.launch.py` unchanged and still works in sim

## Files

| File | Role |
|------|------|
| `launch/nav2_rtabmap_exploration.launch.py` | Full phased stack |
| `launch/nav2_rtabmap_navigation.launch.py` | Nav2 only (no RTAB) |
| `launch/rtabmap_real.launch.py` | RTAB-Map wrapper |
| `config/nav2_params_rtabmap_real.yaml` | Nav2 params, `base_link` |
| `config/rtabmap_real_params.yaml` | RTAB topic reference |
| `config/frontier_params_moonmapper.yaml` | Frontier explorer |
| `moonmapper_bringup/launch/real_robot_navigation.launch.py` | Top-level alias |
