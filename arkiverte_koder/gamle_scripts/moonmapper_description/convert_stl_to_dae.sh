#!/usr/bin/env bash
# convert_stl_to_dae.sh
#
# Konverterer alle STL-meshes i meshes/visual/ til DAE (Collada).
# Unity URDF-Importer har en kjent chdir-bug med VHACD + STL paa Linux;
# DAE/OBJ-stien er ikke rammet. Ved samtidig export med
# export_urdf_for_unity.sh --dae skriver vi en URDF-variant som
# refererer til DAE-filene.
#
# Avhengigheter:
#     sudo apt install assimp-utils
#
# Bruk:
#     bash src/moonmapper_description/scripts/convert_stl_to_dae.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
VISUAL_DIR="$PKG_DIR/meshes/visual"
DAE_DIR="$VISUAL_DIR/dae"

if ! command -v assimp >/dev/null 2>&1; then
    echo "FEIL: 'assimp' er ikke installert." >&2
    echo "       sudo apt install assimp-utils" >&2
    exit 1
fi

mkdir -p "$DAE_DIR"

count=0
for stl in "$VISUAL_DIR"/*.stl; do
    [[ -f "$stl" ]] || continue
    base="$(basename "$stl" .stl)"
    dae="$DAE_DIR/$base.dae"
    echo "  $base.stl  ->  dae/$base.dae"
    assimp export "$stl" "$dae" >/dev/null
    count=$((count + 1))
done

echo
echo "== Ferdig: $count STL-filer konvertert til DAE =="
echo "Plassering: $DAE_DIR"
echo
echo "Neste steg:"
echo "  bash $SCRIPT_DIR/export_urdf_for_unity.sh --dae"
