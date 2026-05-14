#!/usr/bin/env bash
# Bygg hele workspace med riktig ROS 2-underlag (pakraevd for ament_cmake / ament_python).
#
# Uten "source /opt/ros/jazzy/setup.bash" foer colcon kan du faa:
#   ModuleNotFoundError: No module named 'ament_package'
#
# Bruk:
#   ./colcon_build.sh
#   ./colcon_build.sh --packages-select moonmapper_bringup moonmapper_description
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
JAZZY="/opt/ros/jazzy/setup.bash"
if [[ ! -f "$JAZZY" ]]; then
  echo "Fant ikke $JAZZY — installer ROS 2 Jazzy eller juster sti i skriptet." >&2
  exit 1
fi
# shellcheck source=/dev/null
source "$JAZZY"
cd "$ROOT"
colcon build --symlink-install "$@"
echo ""
echo "Neste steg (samme terminal, eller legg i ~/.bashrc):"
echo "  source $JAZZY"
echo "  source $ROOT/install/setup.bash"
