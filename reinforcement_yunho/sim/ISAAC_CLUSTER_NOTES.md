# Isaac Sim on the RTX PRO 6000 cluster node — status & turnkey recipe

**Status (2026-07-17): BLOCKED by the host NVIDIA driver, not the hardware.**
Everything below works; Isaac Sim's RTX renderer crashes at startup ONLY because
of a known driver bug. One admin action unblocks it.

## The blocker (NVIDIA-confirmed)
- Host driver **595.71.05** (R590 branch, CUDA 13.2) is a **known-broken branch for
  the Omniverse RTX renderer on Blackwell GPUs (sm_120 / RTX PRO 6000)**.
- Symptom: fatal crash in `librtx.scenedb.plugin` / `carb.scenerenderer-rtx` right
  after "app ready", before any frame renders. Reproduced on both Isaac Sim 4.5.0
  and 5.0.0 here.
- Confirmed by NVIDIA staff on the exact driver (595.71.05) and the exact GPU
  (RTX PRO 6000 Blackwell): forums.developer.nvidia.com t/366252, t/370054;
  github.com/isaac-sim/IsaacSim discussions/648, issues/517, issues/537.

## The fix (needs root on the RHEL8 host — ask the cluster admin)
> Please downgrade the NVIDIA driver on this node from **595.71.05** to
> **580.65.06** (the Isaac Sim 5.1.0 validated Linux driver; RTX PRO 6000 Blackwell
> is an officially supported GPU on it). The 595.x branch is NVIDIA-confirmed
> broken with the Omniverse RTX renderer on Blackwell.

Driver is passed into the container by `apptainer --nv` from the host, so it must
be changed on the host — it cannot be fixed inside the image or with launch flags.
Also switch the image to `nvcr.io/nvidia/isaac-sim:5.1.0` (5.1 validates Blackwell;
5.0 doesn't). Ref: docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/requirements.html

## What already works here (turnkey once the driver is 580.65.06)
`apptainer pull` can't build a SIF on this node (no /etc/subuid → proot fallback →
seccomp/loader errors). Bypass = extract the cached OCI layers to a plain rootfs and
run it (runtime user namespaces work):

```bash
# 1) layers are cached by an apptainer pull attempt; extract to a rootfs dir
#    (see /scratch/pcn3tv/extract_rootfs5.sh — tar each layer in order + apply .wh. whiteouts)
# 2) run headless with the required env:
source /etc/profile.d/modules.sh; module load apptainer/1.5.0
R=/scratch/pcn3tv/isaac5_rootfs        # extracted rootfs
apptainer exec --nv \
  --env LD_LIBRARY_PATH=/.singularity.d/libs:/usr/lib/x86_64-linux-gnu \
  --env VK_ICD_FILENAMES=/etc/vulkan/icd.d/nvidia_icd.json \
  --env VK_DRIVER_FILES=/etc/vulkan/icd.d/nvidia_icd.json \
  --env HOME=/scratch/pcn3tv/isaac5_home \
  --env ACCEPT_EULA=Y --env OMNI_KIT_ACCEPT_EULA=YES --env PRIVACY_CONSENT=Y \
  --bind /scratch:/scratch,/sfs:/sfs \
  --bind /scratch/pcn3tv/isaac5_cache/kit_cache:/isaac-sim/kit/cache \
  --bind /scratch/pcn3tv/isaac5_cache/kit_data:/isaac-sim/kit/data \
  --bind /scratch/pcn3tv/isaac5_cache/kit_logs:/isaac-sim/kit/logs \
  "$R" /isaac-sim/python.sh sim/isaac_replicator.py --num-frames 3000 --out /scratch/pcn3tv/window_ds
```
Verified working up to the renderer: glibc 2.35 container, GPU visible
(`nvidia-smi` → RTX PRO 6000), single-GPU enumeration (after VK ICD dedup), Kit
starts, `omni.hydra.rtx` loads. Only the RTX device init crashes — the driver bug.

Required env notes:
- `VK_ICD_FILENAMES` dedup: without it, the host + image nvidia ICDs both load →
  GPU enumerated twice → "Failed to create any GPU devices".
- `LD_LIBRARY_PATH=/.singularity.d/libs`: the manually-extracted rootfs lacks
  apptainer's env script, so `--nv`-injected driver libs aren't on the path.
- kit/{cache,data,logs} bound writable: the image dirs are empty and read-only.

## Fallback that does NOT need Isaac (unblocks 길남 Job 1 today)
Generate the dataset with a procedural renderer (CPU) and train `yolo11s-pose` on the
RTX PRO 6000 (torch cu128 supports sm_120 — the GPU is fully usable for *training*,
only its RTX *renderer* is driver-blocked). Not Isaac-photoreal, but it gives 길남 a
real dataset now. See the project README's fallback note.
