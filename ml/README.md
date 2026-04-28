# ML pipeline (utenfor ROS-runtime)

Denne mappen inneholder **trening, datasett, eksperimenter og modell-artefakter** for MoonMapper.

## Hvorfor ligger dette i `ml/`?
- `src/` er reservert for ROS2-pakker som bygges med `colcon` og kjøres som noder i runtime.
- `ml/` er reservert for forskning/utvikling: datasett, feature-ekstraksjon, trening, evaluering og eksperimenter.

## Struktur
- `datasets/`: rådata, prosessert data, eksempler og metadata-template
- `training/`: trenings- og evalueringsskript (skeletons i starten)
- `configs/`: label- og feature-konfigurasjon (YAML)
- `models/`: lagrede modeller (skal normalt ikke inn i git)
- `notebooks/`: Jupyter-notebooks (ikke required for pipeline)

## Første mål (fase 1)
Bygge en **Triad-only baseline** (to AS7265X-sensorer \(\times\) 18 kanaler = 36 kanaler) og trene en enkel Random Forest.

