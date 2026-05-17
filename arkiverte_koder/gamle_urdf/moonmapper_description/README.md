# Arkiverte URDF/Xacro — moonmapper_description

Flyttet 2026-05-17. Ikke brukt av aktiv Gazebo-sim (`gazebo_rover.launch.py` → `moonmapper_rover_gazebo.urdf.xacro`).

| Fil | Grunn |
|-----|--------|
| `moonmapper.urdf.xacro` | Tynn entry; kun `view_robot.launch.py` (arkivert under `gamle_gazebo_tester/`) |
| `moonmapper_macros.xacro` | Ingen `xacro:include` i aktiv rover-URDF |
| `materials.xacro` | Samme — ikke inkludert |

Gjenoppretting: `git mv` tilbake til `src/moonmapper_description/urdf/`.
