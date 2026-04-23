# unity_scripts/

C#-scripts som kjoerer i Unity Editor og Runtime for MoonMapper-rover.
Alle scripts forutsetter at **Unity Robotics Hub ROS-TCP-Connector** er
installert (se `../Unity_setup.md` §3.2).

## Filer

| Fil | Formaal | Plassering i Unity |
|-----|---------|--------------------|
| `JointStatePublisher.cs` | Publiserer `/joint_states` (50 Hz) | Paa `moonmapper`-roten |
| `WheelVelocitySubscriber.cs` | Leser `/unity/wheel_velocities`, driver hjul | Paa `moonmapper`-roten |
| `ImuPublisher.cs` | Publiserer `/imu` (100 Hz) | Paa `imu_link` |
| `CameraImagePublisher.cs` | Publiserer `/<navn>/image_raw` + `camera_info` | Paa et Camera-GameObject under `*_camera_link` |
| `ArenaBoundary.cs` | Definerer 20 m^2 test-omraadet med vegger + Gizmo | Paa tomt GameObject `ArenaBoundary` i scenen |
| `RoverSpawn.cs` | Respawner roveren ved Start og tast `R` | Paa `moonmapper`-roten; refererer til tom `RoverSpawn`-transform |

## Installasjon

1. I Unity Project-panelet, lag mappen `Assets/Scripts/MoonMapper/`.
2. Dra alle `.cs`-filene fra dette katalog inn der.
3. Unity kompilerer automatisk. Sjekk at Console er feilfri.

## Bruk

- ROS-bridge og publishers/subscribers: `../Unity_setup.md` §3.5.
- Maane-arena (Lunar Landscape 3D + 20 m^2 testomraade): `../Unity_arena_setup.md`.

## Ekstensjoner

Hvis du trenger flere publishers (f.eks. `/odom`, `/tf` fra Unity,
`/depth_camera/points`), kan du bruke Unity Robotics Hub's
`com.unity.robotics.visualizations`-pakke eller bygge paa disse
scriptene som maler.
