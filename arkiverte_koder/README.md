# Arkiverte koder — MoonMapper

**Dato:** 2026-05-17  
**Formål:** Bachelorinnlevering — hoved-workspacet inneholder kun aktiv kode, konfigurasjon og dokumentasjon. Ingen filer er slettet; alt som ikke lenger er del av den aktive leveransen er flyttet hit med `git mv` der det var mulig.

## Viktig

- Arkivet er **ikke** en del av standard `colcon build` (ligger utenfor `src/`).
- Gjenoppretting: flytt filer tilbake til opprinnelig sti, eller kjør kommandoer fra arkivmappen.
- Se **`CLEANUP_REPORT.md`** i workspace-roten for full liste over flyttinger.

## Undermapper

| Mappe | Innhold |
|-------|---------|
| `gamle_launch_filer/` | Ubrukte launch-filer (f.eks. `real_robot_navigation`) |
| `gamle_config_filer/` | (reservert — configs ligger ofte under `gamle_nav2_tester/config/`) |
| `gamle_nodes/` | Ubrukte Python-noder (`moonmapper_autonomy`: reactive, coverage, …) |
| `gamle_scripts/` | Diverse hjelpescripts (`tools/test_drive_sanity.sh`) |
| `gamle_plugins/` | (reservert) |
| `gamle_urdf/moonmapper_description/` | `moonmapper.urdf.xacro`, `materials.xacro`, `moonmapper_macros.xacro` |
| `gamle_config/moonmapper_description/` | EKF/jsp-overlay (ikke brukt i autonom stack) |
| `gamle_meshes/moonmapper_description/` | Ubrukte STL (ikke referert i URDF) |
| `gamle_models/moonmapper_description/` | `replan_test_block` (statisk replan-test) |
| `gamle_ml_eksperimenter/` | ML-backup, eksempel-CSVs, lock-filer, testbilder |
| `gamle_sensor_tester/` | Autonomy test-launch (depth/safety/reactive) |
| `gamle_nav2_tester/` | Statisk kart, odom-baseline, utforskning fase 1–3, debug-noder |
| `gamle_gazebo_tester/` | `view_robot.launch.py` |
| `gamle_webots_filer/` | `sim.launch.py` (Webots, pakke fjernet fra `src/`) |
| `gamle_mapping/` | slam_toolbox-pipeline, kart, mission-launch |
| `gamle_ros_pakker/` | Hele pakker: `ros_tcp_endpoint`, `moonmapper_slam`, `moonmapper_control`, `moonmapper_mapping`, `moonmapper_localization`, `moonmapper_gz_sensors`, `moonmapper_msgs`, `moonmapper_interfaces`, `moonmapper_perception`, `moonmapper_ml` |
| `gamle_dokumentasjon_utkast/` | `Kodedokumentasjone.md`, `simulasjon_TEST.md`, gamle package-README |
| `gamle_notebooks/` | `ml_analyse-1.ipynb` |
| `gamle_logger/` | (reservert) |
| `duplikater/` | `FARAH.ino` (duplikat av Arduino-sketch) |
| `midlertidige_debug_filer/` | TF-eksporter, `.deb`, skjermbilder |
| `usikre_filer/` | Filer som ble vurdert, men ikke flyttet (se rapport) |

## Aktiv stack (referanse)

Se hoved-`README.md` i workspace-roten.
