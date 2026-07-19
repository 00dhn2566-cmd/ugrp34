#!/usr/bin/env bash
# Render a window-traversal dataset inside the Isaac Sim 4.5 apptainer container.
#
# Runs sim/isaac_replicator.py with the in-container python entrypoint
# (/isaac-sim/python.sh) on an RTX PRO 6000. Produces:
#     <OUT>/frames/frame_000001.png ...   (RGB, 1280x720)
#     <OUT>/meta/frame_000001.json ...     (per sim/metadata_schema.md)
# then feed <OUT>/meta to sim/export_dataset.py --mode from-metadata (see ISAAC_SETUP.md).
#
# Usage:
#   sim/run_isaac_dataset.sh [NUM_FRAMES] [OUT_DIR] [SEED] [SIF]
# Examples:
#   sim/run_isaac_dataset.sh 500 /scratch/$USER/window_ds 0
#   SIF=/scratch/$USER/isaac-sim_4.5.0.sif sim/run_isaac_dataset.sh 500 /scratch/$USER/window_ds 0
set -euo pipefail

# ---- args (positional, with sensible defaults) -----------------------------
NUM_FRAMES="${1:-100}"
OUT_DIR="${2:-/scratch/$USER/window_dataset}"
SEED="${3:-0}"
# Path to the Isaac Sim 4.5 .sif (pull once, see ISAAC_SETUP.md). Override via 4th
# arg or the SIF env var.
SIF="${4:-${SIF:-/scratch/$USER/isaac-sim_4.5.0.sif}}"

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"   # repo root (reinforcement_yunho)
SCRIPT="$REPO/sim/isaac_replicator.py"

# ---- apptainer module (HPC) ------------------------------------------------
source /etc/profile.d/modules.sh
module load apptainer/1.5.0

# ---- caches on scratch (container image + Kit shader/asset caches are large) ----
export APPTAINER_CACHEDIR="${APPTAINER_CACHEDIR:-/scratch/$USER/.apptainer_cache}"
export APPTAINER_TMPDIR="${APPTAINER_TMPDIR:-/scratch/$USER/.apptainer_tmp}"
mkdir -p "$APPTAINER_CACHEDIR" "$APPTAINER_TMPDIR" "$OUT_DIR"

# Persistent Kit caches so shader compilation isn't repeated every run.
KIT_CACHE="${KIT_CACHE:-/scratch/$USER/isaac_kit_cache}"
mkdir -p "$KIT_CACHE/cache" "$KIT_CACHE/logs" "$KIT_CACHE/data" "$KIT_CACHE/config"

# ---- EULA / headless-render env (Isaac Sim requires these to run unattended) ----
# ACCEPT_EULA + OMNI_KIT_ACCEPT_EULA are mandatory for non-interactive container runs.
export ACCEPT_EULA=Y
export OMNI_KIT_ACCEPT_EULA=YES
export PRIVACY_CONSENT=Y                     # opt out of telemetry prompts
# EGL / offscreen GL for headless RTX on a server with no display:
export DISPLAY=""
export __GLX_VENDOR_LIBRARY_NAME=nvidia
export MESA_GL_VERSION_OVERRIDE=4.6

if [[ ! -f "$SIF" ]]; then
  echo "ERROR: Isaac Sim .sif not found at: $SIF" >&2
  echo "Pull it first (see sim/ISAAC_SETUP.md), e.g.:" >&2
  echo "  apptainer pull \"$SIF\" docker://nvcr.io/nvidia/isaac-sim:4.5.0" >&2
  exit 2
fi

echo "[run_isaac_dataset] SIF=$SIF"
echo "[run_isaac_dataset] REPO=$REPO OUT=$OUT_DIR FRAMES=$NUM_FRAMES SEED=$SEED"

# ---- run ------------------------------------------------------------------
# --nv exposes the NVIDIA GPU/driver. Bind /sfs and /scratch so the repo and the
# output dir are visible inside the container. Bind Kit caches into the container
# HOME (/isaac-sim writes caches under ~/.cache, ~/.nvidia-omniverse, etc.).
apptainer exec --nv \
  --bind /sfs:/sfs \
  --bind /scratch:/scratch \
  --bind "$KIT_CACHE/cache":/root/.cache/ov \
  --bind "$KIT_CACHE/logs":/root/.nvidia-omniverse/logs \
  --bind "$KIT_CACHE/data":/root/.local/share/ov/data \
  --env ACCEPT_EULA="$ACCEPT_EULA" \
  --env OMNI_KIT_ACCEPT_EULA="$OMNI_KIT_ACCEPT_EULA" \
  --env PRIVACY_CONSENT="$PRIVACY_CONSENT" \
  "$SIF" \
  /isaac-sim/python.sh "$SCRIPT" \
    --num-frames "$NUM_FRAMES" \
    --out "$OUT_DIR" \
    --seed "$SEED"

echo "[run_isaac_dataset] frames -> $OUT_DIR/frames , metadata -> $OUT_DIR/meta"
echo "[run_isaac_dataset] next: build the YOLO-pose dataset (no GPU needed):"
echo "  python3 $REPO/sim/export_dataset.py --mode from-metadata \\"
echo "      --metadata-dir $OUT_DIR/meta --out <dataset_root>"
