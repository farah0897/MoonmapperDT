# ML-pipeline oversikt (MoonMapper)

Denne filen beskriver hvordan ML-delen i MoonMapper er strukturert, og hvordan den skal kobles til ROS2 etter hvert.

## Hvorfor ligger ML i `ml/`?
`ml/` er ment for **trening og eksperimentering**:
- datasett (rådata og prosessert data)
- feature-ekstraksjon og preprosessering
- trening og evaluering
- notebooks og eksperimenter
- modell-artefakter (typisk store filer som ikke skal i git)

Dette holdes separat fra ROS-runtime av to hovedgrunner:
- **Ryddighet og byggbarhet**: `src/` bygges med `colcon`. Treningskode/datasett bør ikke påvirke ROS-build.
- **Ulik livssyklus**: trening endres ofte, mens runtime-noder og interfaces skal være stabile.

## Hvorfor ligger ROS-runtime i `src/`?
`src/` er ROS2-workspace sin standardplassering for pakker:
- interface-pakker (`msg`, `srv`, `action`)
- runtime-noder (`rclpy`/`rclcpp`)
- launch-filer, parametre og integrasjon mot resten av systemet

Her er målet at alt kan bygges og kjøres med `colcon build` og `ros2 run`.

## Trening vs inference (runtime)
- **Trening** (i `ml/training/`): tar inn datasett, lærer modellparametre, og lagrer en modellfil (f.eks. `.joblib`).
- **Inference** (i `src/moonmapper_ml/`): laster en ferdigtrent modell og gjør prediksjoner i sanntid på ROS-meldinger.

I praksis:
- trening = “offline” utvikling / eksperimenter
- inference = “online” ROS2 node i roboten

## Fase 1: Triad-only baseline (Random Forest)
Vi starter med to SparkFun AS7265X Triad-spektrosensorer:
- **18 spektralkanaler per sensor**
- **2 sensorer** gir **36 kanaler totalt**
- Vi behandler **begge sensorer separat** (ikke gjennomsnittliggjøre bort informasjon).

Pipeline i fase 1:
- rå burst-data (triad)
- feature-ekstraksjon (mean/std/min/max per kanal + RMS)
- enkel klassifikator (Random Forest)
- output: klasse + confidence

## Fase 2: Computer vision (mikroskopkamera)
Når Triad-baseline fungerer, legger vi til mikroskopkamera:
- lagring/publisering av bilder med metadata-lenking til Triad-målinger
- treningspipeline for en vision-modell (f.eks. CNN/ViT, avhengig av datamengde)

## Fase 3: Sensor fusion (Triad + vision)
Til slutt kombineres modellene:
- en enkel fusjon først (f.eks. weighted average av sannsynligheter)
- senere en egen fusjonsmodell (meta-klassifikator) hvis det gir bedre resultater

## Dataflyt i ROS (målbilde)

### Fase 1 (Triad)
Triad Arduino  
→ `triad_serial_node`  
→ `feature_extraction_node`  
→ `ml_inference_node`  
→ topic: `/ml/classification`

### Fase 2/3 (vision + fusion)
`microscope_camera_node`  
→ vision model  
→ fusion model  
→ final classification

## Notater om datasett og git
- Rådata og prosessert data ligger i `ml/datasets/`, men **skal ikke committes**.
- `ml/datasets/examples/` kan inneholde små eksempel-filer (formatdemo).
- Modeller (`.joblib`, `.pt`, osv.) ignoreres i git som standard.

