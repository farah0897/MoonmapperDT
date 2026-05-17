# MoonMapper Level 1 sensors (Gazebo Sim)

This document is a practical checklist to verify the **Level 1 sensor stack**:

- IMU (`/imu/data`)
- Stereo cameras (`/stereo/*`)
- RGB-D (`/depth/*` and `/depth_camera/*`)
- Microscope camera (`/microscope/*`)
- Triad spectroscopy sensors (synthetic) (`/triad_sensor_{1,2}/spectrum`)

## Launch

Recommended:

```bash
ros2 launch moonmapper_bringup sensors_level1_gz.launch.py
```

## TF sanity

```bash
ros2 run tf2_tools view_frames
```

Expected frames to exist (non-exhaustive):
- `imu_link`
- `stereo_left_optical_frame`
- `stereo_right_optical_frame`
- `depth_camera_optical_frame`
- `mikroskop_optical_frame`
- `triad_sensor_1_link`
- `triad_sensor_2_link`

## IMU

```bash
ros2 topic hz /imu/data
ros2 topic echo /imu/data --once
```

## Stereo cameras

```bash
ros2 topic hz /stereo/left/image_raw
ros2 topic hz /stereo/right/image_raw
ros2 topic echo /stereo/left/camera_info --once
```

Optional visualization:

```bash
ros2 run rqt_image_view rqt_image_view
```

## Depth / RGB-D

Aliases (Level 1 scheme):

```bash
ros2 topic hz /depth/image_raw
ros2 topic hz /depth/image_depth
ros2 topic hz /depth/points
ros2 topic echo /depth/camera_info --once
```

Original (kept for compatibility):

```bash
ros2 topic hz /depth_camera/image
ros2 topic hz /depth_camera/depth_image
ros2 topic hz /depth_camera/points
```

## Microscope camera

```bash
ros2 topic hz /microscope/image_raw
ros2 topic echo /microscope/camera_info --once
```

View it in `rqt_image_view` by selecting the topic `/microscope/image_raw`.

## Triad spectroscopy sensors (synthetic)

```bash
ros2 topic hz /triad_sensor_1/spectrum
ros2 topic echo /triad_sensor_1/spectrum --once
ros2 topic echo /triad_sensor_2/spectrum --once
```

If you drive the rover over different objects (or change collision names / materials in the world),
the `material_label` should switch based on the mapping rules in:

- `moonmapper_gz_sensors/config/triad_material_map.yaml`

