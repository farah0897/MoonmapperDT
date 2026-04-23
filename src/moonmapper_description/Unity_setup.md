# MoonMapper – Unity-simulering med ROS 2

Dette dokumentet beskriver hvordan du setter opp MoonMapper-roveren i
**Unity** som fysikksimulator, koblet til **ROS 2 Jazzy** via
Unity Robotics Hub (ROS-TCP-Connector/Endpoint).

Det gamle Gazebo-oppsettet er arkivert under
`src/moonmapper_description/_archive/gazebo/` og bygges/installeres ikke
lenger.

---

## 1. Arkitektur

```
+------------------+                TCP                +----------------------+
|      ROS 2       |  <------------------------------> |        Unity         |
|  (Jazzy, Linux)  |         port 10000 default        |  (Windows/Linux)     |
+------------------+                                   +----------------------+
 - robot_state_pub    /cmd_vel   -->    /unity/wheel_velocities (Float64Array)
 - unity_cmd_vel_node /joint_states <-- (Unity publiserer)
 - ros_tcp_endpoint   /imu         <-- (Unity simulerer IMU)
 - rviz2              /stereo/left/image_raw  <-- (Unity-kamera)
                      /depth_camera/points    <-- (Unity-depth-kamera)
                      /tf          <-- (fra robot_state_publisher)
```

ROS-siden er **deterministisk og headless**; all fysikk skjer i Unity.

---

## 2. Kom-i-gang – ROS 2-siden

### 2.1 Installer ROS-TCP-Endpoint

ROS-TCP-Endpoint er ikke i apt/rosdep. Klone den inn i workspaceet:

```bash
cd ~/rover/simulasjon/moonmapper_ws/src
git clone -b main-ros2 https://github.com/Unity-Technologies/ROS-TCP-Endpoint ros_tcp_endpoint
```

> `main-ros2`-branchen støtter ROS 2 Humble/Iron/Jazzy.

### 2.2 Bygg pakkene

```bash
cd ~/rover/simulasjon/moonmapper_ws
rosdep install --from-paths src --ignore-src -r -y   # valgfritt
colcon build --symlink-install
source install/setup.bash
```

### 2.3 Start minimal-stacken

```bash
ros2 launch moonmapper_description unity_minimal.launch.py
```

Dette starter:
- `robot_state_publisher`  (publiserer URDF + /tf fra /joint_states)
- `ros_tcp_endpoint`       (lytter på `0.0.0.0:10000`)
- `rviz2`                  (forhåndskonfigurert)

Du skal se denne linjen i loggen:

```
[ros_tcp_endpoint] Starting server on 0.0.0.0:10000
```

### 2.4 Full-stack (med fjernstyring)

```bash
ros2 launch moonmapper_description unity_full.launch.py
```

Starter alt i minimal pluss:
- `unity_cmd_vel_bridge` (`/cmd_vel` → `/unity/wheel_velocities`)
- `teleop_twist_keyboard` i egen xterm-terminal

Sett `start_teleop:=false` hvis du vil bruke andre teleop-kilder:

```bash
ros2 launch moonmapper_description unity_full.launch.py start_teleop:=false
# I egen terminal:
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 2.5 Fri-kjøring (manuell tastaturstyring)

`unity_cmd_vel_bridge` abonnerer på BÅDE `Twist` og `TwistStamped` på
`/cmd_vel`, så samme mønster som Gazebo-stacken bruker fungerer også her.

**Variant A – plain Twist** (default i `unity_full.launch.py`):

```bash
# Kjorer allerede automatisk i egen xterm,
# eller manuelt:
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args \
    -p speed:=0.05 -p turn:=0.2
```

**Variant B – TwistStamped** (samme mønster som Gazebo):

```bash
# Via launch-argumentet:
ros2 launch moonmapper_description unity_full.launch.py stamped:=true

# Eller manuelt:
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args \
    -p stamped:=true \
    -p frame_id:=base_link \
    -p speed:=0.05 -p turn:=0.2
```

**Gazebo-varianten** (etter `restore_gazebo.sh`):

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args \
    -r /cmd_vel:=/diff_drive_controller/cmd_vel \
    -p stamped:=true \
    -p frame_id:=base_link
```

Gazebo-YAMLen har `use_stamped_vel: true` satt eksplisitt, så denne
kommandoen fungerer uten flere justeringer.

Alle tre variantene respekterer hastighetsgrensene i
`config/unity_params.yaml` (hhv. `_archive/gazebo/config/wheel_controllers.yaml`).

---

## 3. Unity-siden

### 3.1 Installér Unity

- Installer **Unity Hub** og et passende Editor (anbefalt: Unity 2022 LTS eller 6000.0 LTS).
- Opprett et nytt 3D-prosjekt (URP eller Built-in, begge funker).

### 3.2 Installér Unity-pakkene

I Unity Editor → `Window → Package Manager → + → Add package from git URL`:

```
https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector
https://github.com/Unity-Technologies/URDF-Importer.git?path=/com.unity.robotics.urdf-importer
```

For sensor-visualisering kan du i tillegg installere:

```
https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.visualizations
```

### 3.3 Konfigurér ROS 2-kobling

`Robotics → ROS Settings`:

| Felt                 | Verdi                                   |
|----------------------|-----------------------------------------|
| ROS IP Address       | IP-en til ROS-maskinen (f.eks. 192.168.1.50) |
| ROS Port             | 10000                                   |
| Protocol             | **ROS 2**                               |
| Connect on Start     | ☑                                        |

### 3.4 Importér URDF-en

URDF-Importer forstår `package://<navn>/...`-URIer som relative stier fra
URDF-filen. Den enkleste varianten er å eksportere en URDF der
`package://`-prefikset er fjernet, slik at meshes ligger rett ved siden
av URDF-filen.

1. **Eksporter Unity-vennlig URDF** fra ROS-workspaceet:

   ```bash
   cd ~/rover/simulasjon/moonmapper_ws
   bash src/moonmapper_description/scripts/export_urdf_for_unity.sh
   # -> skriver unity_export/moonmapper.urdf
   ```

   Scriptet ekspanderer xacro-en med `use_gazebo:=false` og fjerner
   `package://moonmapper_description/` fra alle mesh-stier.

2. **Legg filene i Unity** med følgende struktur:

   ```
   Assets/URDF/
     moonmapper.urdf              <-- fra unity_export/
     meshes/                       <-- kopi av src/moonmapper_description/meshes/
       visual/
         robot_base.stl
         ...
   ```

3. I Unity: høyreklikk `moonmapper.urdf` → **Import Robot from Selected URDF file**.
   - Axis: **Y Axis** (forward)
   - Mesh Decomposer: **None** ← VIKTIG paa Linux med STL-filer
   - Overwrite Existing Prefabs: valgfri
   - La Import velge **ArticulationBody** (ikke Rigidbody).

> **NB om Mesh Decomposer = VHACD:** URDF-Importer har en kjent bug paa
> Linux der VHACD internt gjoer `chdir()` til mesh-filens katalog uten
> aa sette CWD tilbake. Neste `AssetDatabase.CreateAsset(...)` krasjer
> da Unity med "Fatal Error! The current working directory was changed
> from your Unity project folder...". Velg **None** og du unngaar dette
> helt. Da brukes mesh-en selv som collider (med Convex=true der
> noedvendig), noe som holder for en liten, enkel rover som denne.
>
> Hvis du VIL ha VHACD-decomposition, konverter STL -> DAE foerst
> (se 3.4.1 under).

#### 3.4.1 Valgfritt: konverter STL → DAE for korrekt VHACD

Hvis du trenger proper convex-decomposition-colliders (mange separate
ikke-konvekse deler, mjuke kollisjoner), unngaa STL paa Linux og bruk
DAE istedet:

```bash
# Engangs-install
sudo apt install assimp-utils

# Konverter alle STL -> DAE
bash src/moonmapper_description/scripts/convert_stl_to_dae.sh
# -> lager meshes/visual/dae/*.dae

# Regenerer URDF med DAE-stier
bash src/moonmapper_description/scripts/export_urdf_for_unity.sh --dae
```

Kopier saa `meshes/visual/dae/` til `Assets/URDF/meshes/visual/dae/` i
Unity. URDF-en peker til `meshes/visual/dae/<navn>.dae`, og du kan
trygt bruke **VHACD** som Mesh Decomposer.

#### Feilsøking: import-feil

**"DirectoryNotFoundException" → fil ikke funnet**
Feilmelding som `.../moonmapper_description/meshes/visual/robot_base.stl`
betyr at du brukte en URDF med originale `package://`-stier uten
å speile pakkestrukturen. Du har to valg:
- **A)** Bruk `export_urdf_for_unity.sh` (anbefalt) — se steg 1 over.
- **B)** Behold `package://`-stier men lag mappestrukturen Unity
  forventer:
  ```
  Assets/URDF/
    moonmapper.urdf
    moonmapper_description/      <-- samme navn som ROS-pakken
      meshes/visual/*.stl
  ```

**"Fatal Error! The current working directory was changed..."**
VHACD-bug-en beskrevet over. Fiks:
1. Klikk **Quit** paa dialogen, aapne Unity paa nytt.
2. Slett `moonmapper`-noden i Hierarchy + alle `*_NN.asset`-filer under
   `Assets/URDF/meshes/visual/` (auto-genererte VHACD-deler).
3. Re-importer med **Mesh Decomposer = None**, eller foelg 3.4.1 for
   aa bytte til DAE.

**Import-dialogen henger paa "N/24 Links Loaded"**
Lukk dialogen med X. Som regel har det oppstaatt en stille feil lenger
nede. Aapne Console (`Window -> General -> Console`) for aa se hele
stack-tracen, og lukk/gjenaapne Unity for aa rydde tilstand.

### 3.5 Sett opp publishers og subscribers i Unity

ROS-TCP-Connector har ikke en "drag-and-drop"-publisher-komponent – du
må skrive små C#-scripts som ringer `ROSConnection.Publish(...)`.
Ferdige scripts for MoonMapper ligger under
`src/moonmapper_description/unity_scripts/`:

| Fil | Hva det gjør |
|-----|--------------|
| `JointStatePublisher.cs` | `/joint_states` @ 50 Hz |
| `ImuPublisher.cs` | `/imu` @ 100 Hz (med ROS-koordinat-konvertering) |
| `CameraImagePublisher.cs` | `/<navn>/image_raw` + `/<navn>/camera_info` @ 15 Hz |
| `WheelVelocitySubscriber.cs` | Abonnerer på `/unity/wheel_velocities` og driver hjulene |

#### 3.5.0 Legg scripts inn i Unity-prosjektet

1. I Unity Project-panelet: `Assets/` → høyreklikk → **Create → Folder** → navn `Scripts`.
2. Inni `Scripts/`: **Create → Folder** → navn `MoonMapper`.
3. I en filutforsker, kopier alle `.cs`-filene fra
   `~/rover/simulasjon/moonmapper_ws/src/moonmapper_description/unity_scripts/`
   INN i `Assets/Scripts/MoonMapper/`.
4. Unity kompilerer automatisk. Åpne Console (`Window → General → Console`)
   og sjekk at du ikke har røde feil.

#### 3.5.1 ROSConnection-objektet

`ROSConnection` er en singleton som opprettes automatisk første gang et
script kjører `ROSConnection.GetOrCreateInstance()`. Du trenger altså
ikke legge den inn manuelt — men det er nyttig å **ha den synlig i
Hierarchy**:

- `Robotics → ROS Settings` → klikk **Show HUD** slik at du ser status
  oppe-til-venstre i Game-view.
- Verdien `Connected to ROS` viser at ROS-TCP-Endpoint er nådd.

#### 3.5.2 JointStatePublisher (`/joint_states`) — **KREVD**

Uten denne får ikke `robot_state_publisher` oppdatert `/tf`, og RViz
viser den statiske URDF-modellen istedenfor den simulerte.

1. I Hierarchy, klikk på `moonmapper`-roten (importerte URDF-en).
2. I Inspector: **Add Component → Scripts → Joint State Publisher**.
3. På komponenten: høyreklikk tittelen → **Auto Populate From Children**.
4. Verifiser at lista fylles med ~23 ArticulationBody-er
   (alle joints unntatt roten `base_link`).
5. Velg gjerne `Topic Name = "joint_states"` (uten ledende slash —
   det gir konsistent navn både med og uten namespace).

#### 3.5.3 WheelVelocitySubscriber (`/unity/wheel_velocities`) — **KREVD**

Dette er den "ros2_control-erstatningen" som tar imot hjul-kommando fra
`unity_cmd_vel_node.py`.

1. Klikk `moonmapper`-roten i Hierarchy.
2. **Add Component → Wheel Velocity Subscriber**.
3. Sett `Wheel Joints`-array-en til **størrelse 6**.
4. Dra inn hjul-ArticulationBody-ene i *nøyaktig* denne rekkefølgen:
   - `[0]` = `wheel_l1_link` (ArticulationBody)
   - `[1]` = `wheel_l2_link`
   - `[2]` = `wheel_l3_link`
   - `[3]` = `wheel_r1_link`
   - `[4]` = `wheel_r2_link`
   - `[5]` = `wheel_r3_link`
5. `Damping = 10`, `Force Limit = 1` er gode startverdier. Øk
   `Force Limit` hvis hjulene ikke greier å snu under last.

> **Tips:** Hvis roveren kjører baklengs når du holder fram-tasten,
> har du byttet om venstre/høyre. Swap indeks [0-2] og [3-5] i Inspector.

#### 3.5.4 ImuPublisher (`/imu`) — anbefalt

1. I Hierarchy, utvid `moonmapper` → finn `imu_link`-GameObject.
2. Klikk den → **Add Component → Imu Publisher**.
3. `Frame Id = imu_link` (matches URDF-en).
4. La standardverdiene stå (topic `"imu"`, 100 Hz, støy=0).

Scriptet konverterer fra Unity-koordinater (X-høyre, Y-opp, Z-fram)
til ROS REP-103 (X-fram, Y-venstre, Z-opp) internt.

#### 3.5.5 CameraImagePublisher — RGB-kameraer

For stereokameraet skal du ha to instanser, ett per kamera-link:

**Venstre stereokamera:**

1. Utvid `moonmapper` i Hierarchy → finn `stereo_left_camera_link`.
2. Høyreklikk den → **Create → Camera** (oppretter et barn-objekt kalt `Camera`).
3. Sett `Transform → Rotation = (-90, 0, 0)` slik at kameraet ser
   framover i ROS-forstand (Unity-kameraet peker +Z, men må rotere
   for å matche `_optical_frame`-konvensjonen).
4. På det nye Camera-objektet: **Add Component → Camera Image Publisher**.
5. Sett:
   - `Image Topic = stereo/left/image_raw`
   - `Camera Info Topic = stereo/left/camera_info`
   - `Frame Id = stereo_left_optical_frame`
   - `Width = 320`, `Height = 240`, `Publish Rate = 15`.
6. I Camera-komponenten: `Field of View = 70` (matcher Gazebo-varianten).

**Høyre stereokamera:** repeter under `stereo_right_camera_link` med
`.../right/...`-topics og `Frame Id = stereo_right_optical_frame`.

**Depth-kameraet (RGB-delen):** repeter under `depth_camera_link` med
`depth_camera/image` og `Frame Id = depth_camera_optical_frame`.

> **Depth-bilde og PointCloud2** krever litt mer (shader for depth-
> lesing + en PointCloud-konstruksjon). Hvis du trenger det, si fra —
> vi kan utvide `CameraImagePublisher.cs` med en `DepthMode`-flagg.

#### 3.5.6 Topic-oversikt

Etter at alle scripts er på plass og Play er trykket:

| Topic                       | Msg-type                      | Retning | Script                  |
|-----------------------------|-------------------------------|---------|-------------------------|
| `/joint_states`             | `sensor_msgs/JointState`      | U → R   | JointStatePublisher     |
| `/imu`                      | `sensor_msgs/Imu`             | U → R   | ImuPublisher            |
| `/stereo/left/image_raw`    | `sensor_msgs/Image`           | U → R   | CameraImagePublisher    |
| `/stereo/left/camera_info`  | `sensor_msgs/CameraInfo`      | U → R   | CameraImagePublisher    |
| `/stereo/right/image_raw`   | `sensor_msgs/Image`           | U → R   | CameraImagePublisher    |
| `/stereo/right/camera_info` | `sensor_msgs/CameraInfo`      | U → R   | CameraImagePublisher    |
| `/depth_camera/image`       | `sensor_msgs/Image`           | U → R   | CameraImagePublisher    |
| `/depth_camera/camera_info` | `sensor_msgs/CameraInfo`      | U → R   | CameraImagePublisher    |
| `/unity/wheel_velocities`   | `std_msgs/Float64MultiArray`  | R → U   | WheelVelocitySubscriber |

Rekkefølgen i `/unity/wheel_velocities.data` er:
`[wheel_l1, wheel_l2, wheel_l3, wheel_r1, wheel_r2, wheel_r3]` i **rad/s**.

### 3.6 Frame-konvensjoner

ROS bruker `REP-103`: **X framover, Y venstre, Z opp**.
Unity bruker: **X høyre, Y opp, Z framover**.

URDF-Importer håndterer dette automatisk på geometri-siden, men for
Imu/Odom må du transformere manuelt i publish-scriptet. Se
[Unity Robotics Hub-eksemplene](https://github.com/Unity-Technologies/Unity-Robotics-Hub/tree/main/tutorials)
for referanse.

---

## 4. Test-workflow

1. Start ROS-siden: `ros2 launch moonmapper_description unity_full.launch.py`
2. Trykk Play i Unity. I loggen skal du se
   `[ros_tcp_endpoint] Connection to Unity established`.
3. Sjekk at Unity publiserer:
   ```bash
   ros2 topic list | grep -E "joint_states|imu|stereo|depth"
   ros2 topic hz /imu            # skal være ~100 Hz
   ros2 topic hz /joint_states   # skal være ~50 Hz
   ```
4. Kjør roveren via teleop-vinduet (piltaster/WASD avhengig av
   `teleop_twist_keyboard`-bindings).
5. I RViz, sjekk at:
   - Robot-modellen beveger seg riktig (TF fra /joint_states).
   - `/depth_camera/points` viser punktsky (husk QoS = Best Effort).
   - `/stereo/left/image_raw` viser live-bilde.

---

## 5. Feilsøking

### Unity kobler ikke til

- Sjekk at `ros_tcp_endpoint` faktisk kjører: `ros2 node list | grep ros_tcp`.
- Sjekk at brannmur tillater port 10000:
  ```bash
  sudo ufw allow 10000/tcp
  ```
- Hvis Unity og ROS er på **samme maskin**: bruk `127.0.0.1` i Unity ROS Settings.

### Ingen /joint_states fra Unity
- Unity URDF-Importer må importere som **ArticulationBody**, ikke Rigidbody.
- Kontroller at hver joint har navn identisk med URDF-joint (case-sensitivt).

### RViz viser "Fixed Frame [base_footprint] doesn't exist"
- Sjekk at `robot_state_publisher` kjører og at Unity publiserer `/joint_states`
  (ellers publiseres bare statiske TF-er).

### /cmd_vel gjør ingenting
- Sjekk at Unity abonnerer på `/unity/wheel_velocities` (**ikke** `/cmd_vel`).
- Sjekk at `unity_cmd_vel_bridge` publiserer:
  ```bash
  ros2 topic echo /unity/wheel_velocities
  ```

---

## 6. Arkiverte Gazebo-filer

Alt det gamle Gazebo-oppsettet er flyttet til
`src/moonmapper_description/_archive/gazebo/`:

- `urdf/moonmapper_rover_gazebo.urdf.xacro`
- `launch/gazebo_rover.launch.py`
- `worlds/rocker_bogie_test.sdf`
- `config/ros_gz_bridge.yaml`, `config/wheel_controllers.yaml`
- `gz_plugins/`, `hooks/`
- `Gazebo_test.md`

Filene er IKKE lenger med i `install/` eller `CMakeLists.txt`.

### Gjenopplive Gazebo midlertidig

Det finnes to toggle-script under `scripts/`:

```bash
# Bytt til Gazebo-oppsett (flytter filer ut av _archive + patcher package.xml/CMakeLists)
bash src/moonmapper_description/scripts/restore_gazebo.sh
colcon build --packages-select moonmapper_description --symlink-install
source install/setup.bash
ros2 launch moonmapper_description gazebo_rover.launch.py

# Bytt tilbake til Unity-oppsett
bash src/moonmapper_description/scripts/archive_gazebo.sh
colcon build --packages-select moonmapper_description --symlink-install
source install/setup.bash
ros2 launch moonmapper_description unity_minimal.launch.py
```

Systempakkene for Gazebo (`ros-jazzy-ros-gz-*`, `libgz-sim8`) må være
installert paa maskinen. Disse fjernes ikke av arkiveringen, saa du
trenger som regel ikke reinstallere noe.
