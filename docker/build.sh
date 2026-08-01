#!/bin/bash
# Build the solver images. Tags must match execution.OPENFOAM_IMAGE and
# execution.SU2_IMAGE, or set AERO_OPENFOAM_IMAGE / AERO_SU2_IMAGE to match
# whatever you build instead.
#
# The SU2 image compiles from source and takes a while on first build; the
# layer caches, so it happens once per version bump rather than per run.
#
#   ./docker/build.sh            # both
#   ./docker/build.sh openfoam   # just one
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

OPENFOAM_IMAGE="${AERO_OPENFOAM_IMAGE:-aero-drag-tool/openfoam:13}"
SU2_IMAGE="${AERO_SU2_IMAGE:-aero-drag-tool/su2:8.4.0}"

target="${1:-all}"

if [[ "$target" == "all" || "$target" == "openfoam" ]]; then
    echo "==> building $OPENFOAM_IMAGE"
    docker build -f "$here/Dockerfile.openfoam" -t "$OPENFOAM_IMAGE" "$here"
fi

if [[ "$target" == "all" || "$target" == "su2" ]]; then
    echo "==> building $SU2_IMAGE (compiles SU2; expect this one to be slow)"
    docker build -f "$here/Dockerfile.su2" -t "$SU2_IMAGE" "$here"
fi

echo
echo "Done. Check the tool picks them up with:"
echo "    python src/runner.py info"
