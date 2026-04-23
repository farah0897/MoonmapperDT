#!/usr/bin/env bash
# export_urdf_for_unity.sh
#
# Genererer en Unity-vennlig URDF fra xacro-kilden:
#   * Fjerner `package://moonmapper_description/` fra mesh-stier slik at
#     Unity URDF-Importer finner filene direkte ved siden av URDF-en.
#   * Med `--dae`: peker mesh-stiene til meshes/visual/dae/*.dae.
#     Bruk dette hvis du har konvertert STL til DAE med
#     convert_stl_to_dae.sh (unngaar VHACD+STL-bug paa Linux).
#   * Skriver resultatet til et default output under workspace-roten, men
#     du kan gi en annen sti som andre argument.
#
# Bruk:
#     bash export_urdf_for_unity.sh
#     bash export_urdf_for_unity.sh --dae
#     bash export_urdf_for_unity.sh --dae /tmp/moonmapper_dae.urdf
#
# Deretter i Unity:
#     1. Lag en mappe Assets/URDF/
#     2. Kopier URDF-filen dit
#     3. Kopier hele meshes/-mappen fra src/moonmapper_description/
#        til Assets/URDF/meshes/
#     4. Hoyreklikk URDF-en -> Import Robot from Selected URDF file

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
# PKG_DIR = moonmapper_ws/src/moonmapper_description
# PKG_DIR/../.. = moonmapper_ws (workspace-rot)
WS_ROOT="$(cd "$PKG_DIR/../.." && pwd)"

USE_DAE=false
if [[ "${1:-}" == "--dae" ]]; then
    USE_DAE=true
    shift
fi

OUTPUT="${1:-$WS_ROOT/unity_export/moonmapper.urdf}"

# Sorg for at install/ er sourcet, saa xacro finner pakken.
# colcon setup.bash bruker ubound COLCON_TRACE; skru av nounset midlertidig.
if [[ -z "${AMENT_PREFIX_PATH:-}" ]]; then
    if [[ -f "$WS_ROOT/install/setup.bash" ]]; then
        set +u
        # shellcheck disable=SC1091
        source "$WS_ROOT/install/setup.bash"
        set -u
    else
        echo "FEIL: AMENT_PREFIX_PATH er tom og $WS_ROOT/install/setup.bash finnes ikke." >&2
        echo "Kjor 'colcon build' foerst." >&2
        exit 1
    fi
fi

XACRO_SRC="$(ros2 pkg prefix moonmapper_description)/share/moonmapper_description/urdf/moonmapper.urdf.xacro"

if [[ ! -f "$XACRO_SRC" ]]; then
    echo "FEIL: fant ikke xacro-kilden: $XACRO_SRC" >&2
    exit 1
fi

mkdir -p "$(dirname "$OUTPUT")"

echo "== Ekspanderer xacro ->" "$OUTPUT"
xacro "$XACRO_SRC" use_gazebo:=false > "$OUTPUT"

echo "== Fjerner 'package://moonmapper_description/' fra mesh-stier"
sed -i 's|package://moonmapper_description/||g' "$OUTPUT"

if [[ "$USE_DAE" == "true" ]]; then
    DAE_DIR="$PKG_DIR/meshes/visual/dae"
    if [[ ! -d "$DAE_DIR" ]]; then
        echo "FEIL: --dae ble oppgitt, men $DAE_DIR finnes ikke." >&2
        echo "       Kjoer: bash $SCRIPT_DIR/convert_stl_to_dae.sh" >&2
        exit 1
    fi
    echo "== Bytter mesh-stier fra .stl til meshes/visual/dae/*.dae"
    # Bytt .stl-fil til dae/<samme-navn>.dae, bevar 'meshes/visual/'-prefiks.
    sed -i -E 's|(meshes/visual/)([^"/]+)\.stl|\1dae/\2.dae|g' "$OUTPUT"
fi

LINK_COUNT="$(grep -c "<link " "$OUTPUT" || true)"
MESH_COUNT="$(grep -c "<mesh " "$OUTPUT" || true)"

echo
echo "== Ferdig =="
echo "URDF:    $OUTPUT"
echo "Linker:  $LINK_COUNT"
echo "Meshes:  $MESH_COUNT"
echo
echo "Neste steg i Unity:"
echo "  1. Opprett Assets/URDF/"
echo "  2. Kopier $OUTPUT dit"
echo "  3. Kopier $PKG_DIR/meshes til Assets/URDF/meshes/"
echo "  4. Hoyreklikk URDF -> Import Robot from Selected URDF file"
