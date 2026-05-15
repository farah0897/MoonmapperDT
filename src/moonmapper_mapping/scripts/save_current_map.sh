#!/usr/bin/env bash
# Lagrer OccupancyGrid (.yaml + .pgm) via nav2_map_server map_saver_cli og
# serialiserer slam_toolbox-posegraf (.posegraph + .data) for localization.
# Kjør mens mapping (slam_toolbox async) er aktiv og /map publiseres.
set -euo pipefail

if ! command -v ros2 >/dev/null 2>&1; then
  echo "Feil: ros2 ikke i PATH. Kjør: source /opt/ros/<distro>/setup.bash && source install/setup.bash"
  exit 1
fi

if ! ros2 pkg prefix moonmapper_mapping >/dev/null 2>&1; then
  echo "Feil: moonmapper_mapping ikke funnet. Bygg workspace (colcon build) og source install/setup.bash."
  exit 1
fi

if ! ros2 pkg prefix nav2_map_server >/dev/null 2>&1; then
  echo "Feil: nav2_map_server ikke funnet (trengs for map_saver_cli)."
  echo "Installer: sudo apt update && sudo apt install ros-${ROS_DISTRO:-jazzy}-nav2-map-server"
  exit 1
fi

NAME="${1:-moonmapper_slam_map}"
# Tillat kun enkel sti-komponent som filnavn (unngå path traversal)
if [[ "$NAME" == *"/"* || "$NAME" == *".."* ]]; then
  echo "Feil: kartnavn skal være ett filnavn uten skråstrek (f.eks. moonmapper_test_map)."
  exit 1
fi

PKG_PREFIX="$(ros2 pkg prefix moonmapper_mapping)"
PKG_SHARE="${PKG_PREFIX}/share/moonmapper_mapping"
MAPS_DIR="${MOONMAPPER_MAPS_DIR:-$PKG_SHARE/maps}"
mkdir -p "$MAPS_DIR"
BASE="${MAPS_DIR}/${NAME}"

USE_SIM="${MOONMAPPER_USE_SIM_TIME:-true}"
if [[ "${USE_SIM}" != "true" && "${USE_SIM}" != "false" ]]; then
  echo "Feil: MOONMAPPER_USE_SIM_TIME må være true eller false (nå: ${USE_SIM})."
  exit 1
fi

echo "=== MoonMapper: lagre kart ==="
echo "Katalog:     ${MAPS_DIR}"
echo "Stamnavn:    ${NAME}"
echo "Occupancy:   ${BASE}.yaml + ${BASE}.pgm"
echo "Posegraf:    ${BASE}.posegraph + ${BASE}.data"
echo "use_sim_time: ${USE_SIM}"
echo

echo "[1/2] map_saver_cli (nav2_map_server) …"
ros2 run nav2_map_server map_saver_cli -t /map -f "${BASE}" --ros-args -p "use_sim_time:=${USE_SIM}"

if [[ ! -f "${BASE}.yaml" || ! -f "${BASE}.pgm" ]]; then
  echo "Feil: forventet ${BASE}.yaml og ${BASE}.pgm etter map_saver_cli."
  exit 1
fi

echo
echo "[2/2] slam_toolbox serialize_map …"
if ! ros2 service type /slam_toolbox/serialize_map &>/dev/null; then
  echo "Feil: tjenesten /slam_toolbox/serialize_map finnes ikke."
  echo "Start mapping med slam_toolbox (f.eks. autonomy_slam_test) før du lagrer."
  exit 1
fi

RESP="$(ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: \"${BASE}\"}")"
echo "${RESP}"
if echo "${RESP}" | grep -qE 'result(=|:)255'; then
  echo "Feil: serialize_map returnerte RESULT_FAILED_TO_WRITE_FILE."
  exit 1
fi

if [[ ! -f "${BASE}.posegraph" || ! -f "${BASE}.data" ]]; then
  echo "Feil: forventet ${BASE}.posegraph og ${BASE}.data etter serialize_map."
  exit 1
fi

echo
echo "Ferdig."
echo "- Åpne PGM i valgfri bildvisning: ${BASE}.pgm"
echo "- Localization (slam_toolbox): bruk samme stamnavn, f.eks.:"
echo "  ros2 launch moonmapper_mapping localization_test.launch.py map_file:=${BASE}.yaml"
