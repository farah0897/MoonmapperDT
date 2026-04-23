# Unity – Bygg måne-arenaet (20 m²)

Dette dokumentet beskriver hvordan du setter opp test-miljøet for
MoonMapper-roveren i Unity, med:

- **Lunar Landscape 3D** (Evgenii Nikolski, gratis Asset Store-pakke) som
  underlag – realistisk måneregolitt og steiner.
- Et eksplisitt **20 m² test-område** (default 5 m × 4 m) med usynlige
  kollider-vegger og et Scene-gizmo for visualisering.
- Et **spawn-punkt** for roveren slik at du kan resette posisjonen med
  `R` under kjøring.

Hovedflyten er ROS 2-uavhengig – Unity leverer fysikken. Se
`Unity_setup.md` for ROS-bridge-oppsett.

---

## 1. Last ned Lunar Landscape 3D

1. [Unity Asset Store – Lunar Landscape 3D](https://assetstore.unity.com/packages/3d/environments/landscapes/lunar-landscape-3d-132614)
2. `Add to My Assets` → åpne Unity.
3. `Window > Package Manager > My Assets` → velg **Lunar Landscape 3D**
   → `Download` → `Import`.
4. Importer alt (~72 MB). Du får mapper under `Assets/Lunar Landscape/`:
   - `Prefabs/` – ferdige måneoverflate-prefabs.
   - `Materials/`, `Textures/` – regolitt-shader.
   - `Scenes/` – demo-scene du kan bruke som referanse.

Hvis du bruker **URP/HDRP** og asset'en ble laget for Built-in, vil
teksturene vise seg lyserosa. Fix: `Edit > Rendering > Materials >
Convert All Built-in to URP` (eller HDRP).

---

## 2. Lag ny scene for arenaet

Anbefalt å ha en egen scene for test-miljøet – da beholder du
demo-scener rene.

1. `File > New Scene > Basic (URP)` (eller Built-in hvis du bruker det).
2. `File > Save As` → `Assets/Scenes/MoonArena.unity`.
3. Slett `Directional Light` hvis Lunar Landscape-prefaben har sitt eget
   lysoppsett (se i demo-scenen).

---

## 3. Plasser måneterrenget

1. I Project: åpne demo-scenen under `Assets/Lunar Landscape/Scenes/` og
   kopier `Terrain`- eller `LunarSurface`-GameObjectet (Ctrl-C, bytt
   scene, Ctrl-V), eller
2. Dra `Prefabs/LunarSurface` (eller tilsvarende) direkte inn i
   `MoonArena.unity`.
3. Plasser i `(0, 0, 0)`. Terrenget har typisk sine egne `Terrain
   Collider` og steiner.

> **Sjekk at Terrain Collider er aktivt.** Uten det vil roveren falle
> rett gjennom bakken.

---

## 4. Sett opp lyssetting for månerealisme

Månen har hard sol og ingen atmosfære – skyggene er nærmest svarte.

1. `Window > Rendering > Lighting > Environment`:
   - **Skybox Material**: Lunar Landscape har et eget stjernehimmel-
     skybox (sjekk `Assets/Lunar Landscape/Materials/Skybox*`). Velg
     den. Alternativt lag et sort material.
   - **Environment Lighting > Source**: `Color`, farge = `#101010`
     (nesten svart) eller `Intensity Multiplier = 0.05`.
2. `GameObject > Light > Directional Light`:
   - Rotation: `(50, -30, 0)` (høy sol, litt skrått fra side).
   - Color: hvit (`#FFFFFF`).
   - Intensity: `2.0` (Built-in) / `130000 lux` (URP) / tilsvarende HDRP.
   - Shadow Type: `Soft Shadows`, strength `1.0`.
3. Slå av `Auto Generate` i Lighting-fanen og trykk `Generate Lighting`
   én gang.

---

## 5. Legg inn 20 m² test-område

Kopier `ArenaBoundary.cs` og `RoverSpawn.cs` fra ROS-prosjektet:

```bash
cp ~/rover/simulasjon/moonmapper_ws/src/moonmapper_description/unity_scripts/ArenaBoundary.cs \
   <din-Unity-prosjektsti>/Assets/Scripts/MoonMapper/
cp ~/rover/simulasjon/moonmapper_ws/src/moonmapper_description/unity_scripts/RoverSpawn.cs \
   <din-Unity-prosjektsti>/Assets/Scripts/MoonMapper/
```

### 5.1 ArenaBoundary
1. I Hierarchy: `Create Empty` → gi navnet `ArenaBoundary`.
2. Flytt den til midten av et FLATT område på terrenget.
   Sett Y ≈ terrenghøyden (bruk `F` for å zoome, og juster med Transform-
   tool).
3. `Add Component → Arena Boundary`.
4. Inspector:
   - `Size`: `X=5, Y=0.3, Z=4` (= 20 m², 30 cm vegghøyde).
   - `Build Walls On Start`: `true`.
   - `Walls Visible`: `false` (kun kollider; sett `true` hvis du vil se
     dem i Game-view).
   - `Spawn Corner Posts`: `true` hvis du vil ha synlige stolper som
     SLAM-landmarks.
5. I Scene-view ser du nå et **oransje wireframe-rektangel** med
   halv-transparent bunn. Det er grensen.

### 5.2 RoverSpawn
1. Under `ArenaBoundary`: `Create Empty` → `RoverSpawn`.
2. Plasser i `(0, 0.10, 0)` relativt til ArenaBoundary (= 10 cm over
   bakken, midt i arenaet). Peiler du ønsket kjøreretning (typisk +Z),
   roter så `forward` stemmer.
3. Legg `RoverSpawn.cs`-scriptet på **`moonmapper`-roten**, ikke på det
   tomme objektet.
4. I Inspector:
   - `Spawn Point`: dra `RoverSpawn`-tomgjenstanden inn.
   - `Rover Root`: dra `moonmapper`-GameObject inn.
   - `Root Body`: dra `ArticulationBody`-komponenten fra `base_link`
     inn.
   - `Spawn On Start`: `true`.
   - `Reset Key`: `R` (eller din foretrukne tast).

---

## 6. Slipp roveren inn på arenaet

1. Dra `moonmapper`-roten så den ligger med senter ved `RoverSpawn`
   (RoverSpawn-scriptet gjør dette automatisk ved Play, men det er pent
   i Edit-view også).
2. Trykk **Play**.
3. Roveren skal:
   - Stå stille på sanden (ingen fall-gjennom).
   - Være innenfor det oransje wireframet.
   - Respondere på `/cmd_vel` (se `Unity_setup.md` §5).
4. Trykk `R` under kjøring for å respawne til utgangspunkt.

---

## 7. Valgfritt – legg inn hindringer

Lunar Landscape inneholder `MoonRock_*`-prefabs:

1. Dra 3–5 rocks inn i Scene, plasser **innenfor** ArenaBoundary-
   rektangelet.
2. Sørg for at hver `MoonRock` har en `MeshCollider` (pakken setter det
   opp selv på de fleste prefabs). Ellers: `Add Component > Mesh
   Collider` med `Convex = false` for nøyaktige konturer.
3. Rocks bør være statiske: huk på `Static`-flagget øverst til høyre i
   Inspector så Unity ikke simulerer bevegelse på dem.

For tyngre tester: mindre kratere kan lages med `Terrain >
Paint Height`-verktøyet direkte på Lunar Landscape sin Terrain.

---

## 8. Justering av arena-størrelse

`ArenaBoundary.Size` kan endres når som helst – Scene-gizmoet
oppdateres live. Usynlige vegger bygges på nytt når du trykker Play.

Typiske varianter:

| Form | Size (X, Y, Z) | Areal |
|------|----------------|-------|
| Rektangel (anbefalt) | `(5, 0.3, 4)` | 20 m² |
| Kvadrat | `(4.47, 0.3, 4.47)` | 20 m² |
| Korridor | `(10, 0.3, 2)` | 20 m² |
| Større utfordring | `(8, 0.3, 6)` | 48 m² |

---

## 9. Feilsøking

| Symptom | Årsak | Fix |
|---------|-------|-----|
| Rover faller gjennom sanda | Terrain Collider mangler | Inspector på `Terrain` → huk på `Terrain Collider` eller legg til |
| Rover spretter vilt ved Play | Spawner under terrenget | Øk `RoverSpawn.verticalOffset` eller flytt RoverSpawn Y opp |
| Lilla teksturer | URP/HDRP-konflikt | `Edit > Rendering > Materials > Convert to URP` |
| Ingen skygger | Ambient lys for sterkt | Sett `Environment Lighting Intensity` til 0.05 eller lavere |
| Vegger synlige i Game | `Walls Visible = true` | Sett til false |
| `R` respawner ikke | `RoverSpawn.rootBody` ikke satt | Dra `ArticulationBody` fra `base_link` inn |

---

## 10. Hva er neste?

Når arenaet står ferdig:

1. Verifiser `unity_full.launch.py` fortsatt kobler seg til Unity.
2. Legg inn `ImuPublisher.cs` på `imu_link` og `CameraImagePublisher.cs`
   på kamera-link (se `Unity_setup.md` §5).
3. Når IMU og kameraer publiserer, er vi klare for localisering/SLAM
   (`moonmapper_localization`, `moonmapper_slam`).
