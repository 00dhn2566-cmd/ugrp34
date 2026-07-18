"""Assemble + split a YOLO-pose window dataset for 길남 (spec §4.3).

WHAT
----
Two modes:

  --mode from-metadata   (runs anywhere: numpy + pyyaml only)
      Read a directory of per-frame metadata JSON (see sim/metadata_schema.md),
      build labels with sim.replicator_writer.build_label_records, and lay out a
      dataset with an 80/10/10 seeded split (spec §4.3):
          <out>/images/{train,val,test}/*.png
          <out>/labels/{train,val,test}/*.txt   (same stem as the image)
          <out>/meta.jsonl                       (길남's eval_corners schema)
          <out>/dataset_manifest.json            (reproducibility)

  --mode isaac           (import-guarded stub; needs Isaac Sim)
      Render frames + write labels directly via the omni.replicator writer.

WHY meta.jsonl MATTERS (real contract with 길남)
------------------------------------------------
길남's overall_gilnam/vision/eval_corners.py distance-bins the corner error by
reading meta.jsonl. Its _load_meta expects, per image, one record:
    {"image": "images/{split}/{name}.png",
     "windows": [{"order_index": <int>, "distance_m": <float>}, ...]}
and keys distance by order_index -> {stem: {order_index: distance_m}}. We write
EXACTLY that schema (matching 길남's make_toy_dataset.py) so 길남 can run
eval_corners on 윤호's real dataset unchanged. ``distance_m`` = ||window_centre -
camera_position|| in world metres, rounded to 3 dp (same as 길남), and ONLY for
the fully-visible (labelled) windows -- meta rows and label lines stay in lock-step.

NOTE (open item, surfaced to 윤호): eval_corners keys distance_m by order_index,
so if a single frame carries two windows of the SAME colour (order_index), its
distance dict keeps only one. Scene_gen may emit up to 5 windows over 3 colours;
confirm with 길남 whether repeated colours per frame are in scope for the eval set.

Reproducibility (CONVENTIONS.md): every run writes a dataset_manifest.json with
seed, intrinsics, source dir, counts, and git commit hash when available.
"""
from __future__ import annotations

import argparse
import glob
import json
import os
import shutil
import subprocess
import sys
from dataclasses import dataclass
from typing import Dict, List, Sequence

import numpy as np
import yaml

# --- shared 'common' + sibling sim modules, regardless of cwd ----------------
_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)
from common import CameraIntrinsics  # noqa: E402
from sim.replicator_writer import build_label_records  # noqa: E402
from sim.scene_gen import window_rotation_from_normal  # noqa: E402

SPLITS = ("train", "val", "test")


# ======================================================================
#  PURE, TESTABLE SPLIT  (unchanged contract: seeded 80/10/10, n=10 -> 8/1/1)
# ======================================================================
def split_indices(
    n: int, seed: int, *, val_frac: float = 0.10, test_frac: float = 0.10
) -> Dict[str, List[int]]:
    """Deterministic 80/10/10 (default) split of ``range(n)`` (CONVENTIONS.md).

    Seeded numpy permutation, so the same (n, seed) always yields the same
    assignment. Counts: n_test = round(n*test_frac), n_val = round(n*val_frac),
    n_train = n - n_val - n_test. For n=10 this is 8/1/1.
    """
    if n < 0:
        raise ValueError(f"n must be >= 0, got {n}")
    rng = np.random.default_rng(seed)
    perm = rng.permutation(n)

    n_val = min(int(round(n * val_frac)), n)
    n_test = min(int(round(n * test_frac)), n - n_val)
    n_train = n - n_val - n_test

    train = sorted(int(i) for i in perm[:n_train])
    val = sorted(int(i) for i in perm[n_train : n_train + n_val])
    test = sorted(int(i) for i in perm[n_train + n_val :])
    return {"train": train, "val": val, "test": test}


# ======================================================================
#  from-metadata MODE  (pure; runs here)
# ======================================================================
@dataclass
class FrameMeta:
    """One rendered frame's metadata (see sim/metadata_schema.md)."""

    stem: str
    image_path: str
    T_world_cam_usd: np.ndarray
    windows: List[dict]


def _resolve_window(w: dict) -> dict:
    """Return a build_label_records-ready window: a bare "normal" is turned into
    an R_world_win via scene_gen.window_rotation_from_normal (single source of the
    normal->frame convention). center/width/height/color are passed through."""
    if "R_world_win" in w or "quat_wxyz" in w:
        return w
    if "normal" in w:
        out = dict(w)
        out["R_world_win"] = window_rotation_from_normal(w["normal"])
        return out
    raise KeyError(
        f"window needs 'R_world_win', 'quat_wxyz', or 'normal'; got keys {list(w)}"
    )


def _load_frame_meta(json_path: str) -> FrameMeta:
    with open(json_path, "r", encoding="utf-8") as f:
        d = json.load(f)
    stem = os.path.splitext(os.path.basename(json_path))[0]
    img = d["image"]
    if not os.path.isabs(img):
        img = os.path.join(os.path.dirname(json_path), img)
    T = np.asarray(d["T_world_cam_usd"], dtype=np.float64).reshape(4, 4)
    windows = [_resolve_window(w) for w in d["windows"]]
    return FrameMeta(stem=stem, image_path=img, T_world_cam_usd=T, windows=windows)


def _git_commit_hash(cwd: str):
    """Best-effort short commit hash for the manifest; None if not a git repo."""
    try:
        out = subprocess.run(
            ["git", "rev-parse", "--short", "HEAD"],
            cwd=cwd, capture_output=True, text=True, timeout=5,
        )
        return out.stdout.strip() if out.returncode == 0 else None
    except Exception:
        return None


def _meta_windows(frame: FrameMeta, records: List[dict]) -> List[dict]:
    """Per-labelled-window {order_index, distance_m} for meta.jsonl (길남 schema).

    distance_m = ||window_centre - camera_position|| (world m, 3 dp). Camera
    position is the translation of T_world_cam_usd (same in USD/CV — only rotation
    differs). Only the windows that produced a label line are included, so meta
    rows align 1:1 with label lines (as in 길남's make_toy_dataset.py)."""
    cam_pos = frame.T_world_cam_usd[:3, 3]
    out = []
    for r in records:
        center = np.asarray(frame.windows[r["window_index"]]["center"], float)
        out.append(
            {
                "order_index": int(r["order_index"]),
                "distance_m": round(float(np.linalg.norm(center - cam_pos)), 3),
            }
        )
    return out


def assemble_from_metadata(
    metadata_dir: str,
    out_dir: str,
    intr: CameraIntrinsics,
    seed: int,
    *,
    drop_empty: bool = False,
    copy_images: bool = True,
) -> Dict[str, object]:
    """Build labels + meta.jsonl for every metadata frame and lay out a split
    dataset. Returns a manifest dict (also written to dataset_manifest.json)."""
    json_paths = sorted(glob.glob(os.path.join(metadata_dir, "*.json")))
    if not json_paths:
        raise FileNotFoundError(f"no *.json metadata found in {metadata_dir!r}")

    frames: List[FrameMeta] = []
    labels: List[List[str]] = []
    metas: List[List[dict]] = []
    for jp in json_paths:
        fm = _load_frame_meta(jp)
        records = build_label_records(fm.windows, fm.T_world_cam_usd, intr)
        if drop_empty and not records:
            continue  # keep only frames with >=1 visible window
        frames.append(fm)
        labels.append([r["line"] for r in records])
        metas.append(_meta_windows(fm, records))

    n = len(frames)
    assignment = split_indices(n, seed)
    # stem -> split so meta.jsonl can name images/<split>/<stem>.png (길남 reads stem)
    split_of = {i: s for s, idxs in assignment.items() for i in idxs}

    for split in SPLITS:
        os.makedirs(os.path.join(out_dir, "images", split), exist_ok=True)
        os.makedirs(os.path.join(out_dir, "labels", split), exist_ok=True)

    counts = {s: 0 for s in SPLITS}
    missing_images: List[str] = []
    meta_path = os.path.join(out_dir, "meta.jsonl")
    with open(meta_path, "w", encoding="utf-8") as meta_f:
        for i, fm in enumerate(frames):
            split = split_of[i]
            counts[split] += 1
            ext = os.path.splitext(fm.image_path)[1] or ".png"
            img_rel = f"images/{split}/{fm.stem}{ext}"
            if copy_images:
                dst = os.path.join(out_dir, img_rel)
                if os.path.exists(fm.image_path):
                    shutil.copy2(fm.image_path, dst)
                else:
                    missing_images.append(fm.image_path)
            lbl_dst = os.path.join(out_dir, "labels", split, fm.stem + ".txt")
            with open(lbl_dst, "w", encoding="utf-8") as f:
                f.write("\n".join(labels[i]) + ("\n" if labels[i] else ""))
            # 길남 eval_corners._load_meta schema (image path is 'images/<split>/<stem>.png')
            meta_f.write(
                json.dumps({"image": img_rel, "windows": metas[i]}, ensure_ascii=False) + "\n"
            )

    manifest = {
        "seed": seed,
        "intrinsics": intr.to_yaml_dict(),
        "metadata_dir": os.path.abspath(metadata_dir),
        "out_dir": os.path.abspath(out_dir),
        "n_frames": n,
        "counts": counts,
        "drop_empty": drop_empty,
        "git_commit": _git_commit_hash(_ROOT),
        "missing_images": missing_images,
        "meta_jsonl": os.path.basename(meta_path),
    }
    with open(os.path.join(out_dir, "dataset_manifest.json"), "w", encoding="utf-8") as f:
        json.dump(manifest, f, indent=2)
    return manifest


# ======================================================================
#  isaac MODE  (import-guarded stub)
# ======================================================================
def assemble_from_isaac(out_dir: str, intr: CameraIntrinsics, seed: int, **kwargs):
    """Render frames + write labels via omni.replicator (Isaac Sim only).

    Stub: wires the WindowCornerWriter into a Replicator graph (see
    sim.scene_gen.build_replicator_graph). Import is guarded so this module still
    loads outside Isaac Sim; split the rendered frames with split_indices exactly
    as from-metadata does, and dump meta.jsonl from the same per-frame GT.
    """
    try:
        import omni.replicator.core as rep  # type: ignore  # noqa: F401
    except Exception as e:
        raise ImportError(
            "isaac mode needs omni.replicator (run inside the Isaac Sim python "
            f"env). Import error: {e!r}. Use --mode from-metadata offline."
        ) from e
    raise NotImplementedError(
        "isaac mode is a scaffold; complete the Replicator graph inside Isaac Sim "
        "(sim.scene_gen.build_replicator_graph), then split with split_indices."
    )


# ======================================================================
#  CLI
# ======================================================================
def _load_intrinsics(path: str) -> CameraIntrinsics:
    with open(path, "r", encoding="utf-8") as f:
        return CameraIntrinsics.from_yaml_dict(yaml.safe_load(f))


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument(
        "--mode", choices=("from-metadata", "isaac"), default="from-metadata",
        help="from-metadata (offline, default) or isaac (needs Isaac Sim).",
    )
    p.add_argument("--metadata-dir", help="dir of per-frame *.json (from-metadata).")
    p.add_argument("--out", required=True, help="dataset root to create.")
    p.add_argument(
        "--intrinsics",
        default=os.path.join(os.path.dirname(_ROOT), "overall_gilnam", "vision", "synth_intrinsics.yaml"),
        help="intrinsics yaml (default: 길남's synth_intrinsics.yaml).",
    )
    p.add_argument("--seed", type=int, default=0, help="split seed (reproducible).")
    p.add_argument(
        "--drop-empty", action="store_true",
        help="skip frames with no visible window (default: keep as negatives).",
    )
    return p


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    intr = _load_intrinsics(args.intrinsics)

    if args.mode == "from-metadata":
        if not args.metadata_dir:
            build_parser().error("--metadata-dir is required for --mode from-metadata")
        manifest = assemble_from_metadata(
            args.metadata_dir, args.out, intr, args.seed, drop_empty=args.drop_empty
        )
        print(json.dumps(manifest["counts"], indent=2))
        print(f"wrote dataset -> {os.path.abspath(args.out)} (+ meta.jsonl for 길남)")
    else:
        assemble_from_isaac(args.out, intr, args.seed)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
