# moonmapper_gz_sensors (arkivert)

**Arkivert:** 2026-05-17

Gazebo Sim 8-plugin `libmoonmapper_triad_spectroscopy.so` — syntetisk Triad-spektroskopi (raycast + `triad_material_map.yaml`).

## Ikke brukt i aktiv stack

Autonom sim og fysisk metall-ID bruker **ikke** syntetisk Triad. Rocker-bogie-plugin ligger i `moonmapper_description/gz_plugins/`.

URDF: `enable_triad_spectroscopy` default `false` i `moonmapper_rover_gazebo.urdf.xacro`.

## Avhengighet

Krever `moonmapper_msgs` (TriadSpectrum) — arkivert ved siden av i `gamle_ros_pakker/moonmapper_msgs/`.

## Gjenoppretting

```bash
git mv arkiverte_koder/gamle_ros_pakker/moonmapper_gz_sensors src/
git mv arkiverte_koder/gamle_ros_pakker/moonmapper_msgs src/
colcon build --packages-select moonmapper_msgs moonmapper_gz_sensors
```

Deretter: `gazebo_rover.launch.py enable_triad_spectroscopy:=true`.
