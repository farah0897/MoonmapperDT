# moonmapper_ml (arkivert)

**Arkivert:** 2026-05-17

Placeholder ROS 2-pakke: `ml_inference_node` (lastet `.joblib`, ingen subscribe/publish).

## Aktiv ML uten ROS

- Trening og inferens: `ml/`, `scripts/predict.py`
- Innsamling: `ml/data_collection/collect_triad_burst.py`
- Hardware: `arduino/moonmapper_triad_logger/`

## Gjenoppretting

```bash
git mv arkiverte_koder/gamle_ros_pakker/moonmapper_interfaces src/
git mv arkiverte_koder/gamle_ros_pakker/moonmapper_perception src/
git mv arkiverte_koder/gamle_ros_pakker/moonmapper_ml src/
colcon build --packages-select moonmapper_interfaces moonmapper_perception moonmapper_ml
```

Avhenger av `moonmapper_interfaces`.
