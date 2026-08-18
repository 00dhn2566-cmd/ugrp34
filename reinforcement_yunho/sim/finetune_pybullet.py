"""Fine-tune the shipped YOLO-pose detector onto the PyBullet open-frame domain.

    python sim/finetune_pybullet.py

WHAT / WHY
----------
``window_yolo11s_best.pt`` was trained on ``sim/procedural_render.py`` frames where
a window is an OPAQUE filled quad, and with ``single_cls=True`` so it never learned
colour. Both hurt on the real task: an opaque window is a wall (the near pane hides
the far ones and HSV colour judging then reads the near window's colour for all of
them), and colour-by-HSV breaks under exactly that occlusion.

So we fine-tune onto open frames and let the network learn colour itself:

  * ``freeze=11``      backbone (5.44 M, 56 %) frozen — it produced keypoint
                       mAP50-95 0.927 and low-level features transfer. Neck + head
                       (4.27 M) adapt to the new look.
  * ``single_cls=False``  nc 1 -> 3 (red/green/blue). The cv3 classification branch's
                       final 1x1 conv is re-initialised, hence the warmup epochs.
  * ``hsv_h=0.0``      hue augmentation MUST be off: colour is now a label, and
                       jittering hue teaches the model to ignore the very cue it
                       needs. (Everything else in the default aug recipe is fine;
                       ``fliplr`` is safe because window_pose_pyb.yaml carries
                       ``flip_idx: [1,0,3,2]`` to swap TL<->TR / BL<->BR.)
  * ``batch=4``        measured optimum on the MX450 (2.15 GB): 1/2/4/8 all fit,
                       and 4 was fastest at 64.6 s/epoch (8 -> 78.9 s as VRAM tightens,
                       16 spills to host memory and takes minutes).

The original weights are never overwritten — output goes to its own run directory.

Curves are mirrored into ``--fig-dir`` after every epoch so training can be watched
from outside the terminal.
"""
from __future__ import annotations

import argparse
import csv
import os
import shutil
import sys

DEFAULT_DATA = "/home/yoonho/models/pyb_ds/window_pose_pyb.yaml"
DEFAULT_WEIGHTS = "/home/yoonho/models/yolo/window_yolo11s_best.pt"
DEFAULT_FIG = "/home/yoonho/fig"
DEFAULT_PROJECT = "/home/yoonho/models/finetune"

# results.csv column -> (panel, label). Ultralytics names vary slightly by task.
_PANELS = [
    ("losses (train)", [("train/box_loss", "box"), ("train/pose_loss", "pose"),
                        ("train/cls_loss", "cls"), ("train/kobj_loss", "kobj")]),
    ("losses (val)",   [("val/box_loss", "box"), ("val/pose_loss", "pose"),
                        ("val/cls_loss", "cls"), ("val/kobj_loss", "kobj")]),
    ("box metrics",    [("metrics/precision(B)", "precision"), ("metrics/recall(B)", "recall"),
                        ("metrics/mAP50(B)", "mAP50"), ("metrics/mAP50-95(B)", "mAP50-95")]),
    ("pose metrics",   [("metrics/precision(P)", "precision"), ("metrics/recall(P)", "recall"),
                        ("metrics/mAP50(P)", "mAP50"), ("metrics/mAP50-95(P)", "mAP50-95")]),
]


def plot_curves(csv_path: str, out_png: str) -> None:
    """Redraw the training curves from results.csv (English labels only)."""
    if not os.path.exists(csv_path):
        return
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    with open(csv_path) as f:
        rows = list(csv.DictReader(f))
    if not rows:
        return
    cols = {k.strip(): [float(r[k]) for r in rows if r[k] not in ("", None)]
            for k in rows[0]}
    ep = cols.get("epoch", list(range(1, len(rows) + 1)))

    fig, axes = plt.subplots(2, 2, figsize=(13, 8))
    for ax, (title, series) in zip(axes.ravel(), _PANELS):
        drew = False
        for key, label in series:
            y = cols.get(key)
            if y:
                ax.plot(ep[:len(y)], y, lw=1.8, label=label)
                drew = True
        ax.set_title(title, fontsize=11)
        ax.set_xlabel("epoch")
        ax.grid(alpha=.3)
        if drew:
            ax.legend(fontsize=8)
    fig.suptitle("YOLO11s-pose fine-tune on PyBullet open-frame windows "
                 "(freeze=11, 3 classes)", fontsize=13)
    fig.tight_layout()
    os.makedirs(os.path.dirname(out_png), exist_ok=True)
    fig.savefig(out_png, dpi=120)
    plt.close(fig)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--data", default=DEFAULT_DATA)
    ap.add_argument("--weights", default=DEFAULT_WEIGHTS)
    ap.add_argument("--epochs", type=int, default=40)
    ap.add_argument("--batch", type=int, default=4)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--freeze", type=int, default=11)
    ap.add_argument("--lr0", type=float, default=0.001)
    ap.add_argument("--device", default="0")
    ap.add_argument("--workers", type=int, default=2)
    ap.add_argument("--patience", type=int, default=12)
    ap.add_argument("--name", default="pyb_openframe")
    ap.add_argument("--project", default=DEFAULT_PROJECT)
    ap.add_argument("--fig-dir", default=DEFAULT_FIG)
    ap.add_argument("--resume", action="store_true",
                    help="run dir 의 last.pt 에서 이어서 학습 (WSL 끊김 등으로 죽었을 때). "
                         "체크포인트에 저장된 하이퍼파라미터를 쓰므로 다른 인자는 무시된다.")
    a = ap.parse_args(argv)

    from tqdm import tqdm
    from ultralytics import YOLO

    run_dir = os.path.join(a.project, a.name)
    fig_png = os.path.join(a.fig_dir, "finetune_curves.png")
    os.makedirs(a.fig_dir, exist_ok=True)

    print(f"data     {a.data}")
    print(f"weights  {a.weights}")
    print(f"freeze   {a.freeze}  (backbone frozen, neck+head trainable)")
    print(f"batch    {a.batch}   imgsz {a.imgsz}   lr0 {a.lr0}   epochs {a.epochs}")
    print(f"run dir  {run_dir}")
    print(f"curves   {fig_png}  (refreshed every epoch)\n")

    last_pt = os.path.join(run_dir, "weights", "last.pt")
    if a.resume:
        if not os.path.exists(last_pt):
            print(f"[error] 이어서 돌릴 체크포인트가 없다: {last_pt}", file=sys.stderr)
            return 2
        done = 0
        csv_path = os.path.join(run_dir, "results.csv")
        if os.path.exists(csv_path):
            with open(csv_path) as f:
                done = max(0, sum(1 for _ in f) - 1)
        print(f"resume   {last_pt}  ({done} 에폭 완료 -> {a.epochs} 까지)\n")
        model = YOLO(last_pt)
    else:
        model = YOLO(a.weights)

    bar = tqdm(total=a.epochs, desc="epochs", unit="ep", dynamic_ncols=True,
               position=1, leave=True)

    def on_epoch_end(trainer):
        plot_curves(str(trainer.csv), fig_png)
        m = getattr(trainer, "metrics", {}) or {}
        bar.set_postfix({
            "box_mAP50": f"{m.get('metrics/mAP50(B)', float('nan')):.3f}",
            "pose_mAP50": f"{m.get('metrics/mAP50(P)', float('nan')):.3f}",
        })
        bar.update(1)

    model.add_callback("on_fit_epoch_end", on_epoch_end)

    if a.resume:
        # ultralytics 는 resume=True 면 체크포인트의 train_args 를 그대로 복원한다.
        # 여기서 다른 인자를 같이 주면 무시되거나 충돌하므로 resume 만 넘긴다.
        model.train(resume=True)
    else:
        model.train(
            data=a.data, epochs=a.epochs, imgsz=a.imgsz, batch=a.batch,
            freeze=a.freeze, single_cls=False,
            device=a.device, workers=a.workers,
            optimizer="AdamW", lr0=a.lr0, lrf=0.01, cos_lr=True, warmup_epochs=3,
            hsv_h=0.0,                  # colour is a label now — never jitter hue
            patience=a.patience,
            project=a.project, name=a.name, exist_ok=True,
            plots=True, verbose=True,
        )
    bar.close()

    # final curves + copy ultralytics' own plots next to them
    plot_curves(os.path.join(run_dir, "results.csv"), fig_png)
    copied = []
    for fn in ("results.png", "confusion_matrix_normalized.png", "confusion_matrix.png",
               "PR_curve.png", "F1_curve.png", "labels.jpg",
               "val_batch0_labels.jpg", "val_batch0_pred.jpg"):
        src = os.path.join(run_dir, fn)
        if os.path.exists(src):
            dst = os.path.join(a.fig_dir, f"finetune_{fn}")
            shutil.copy(src, dst)
            copied.append(os.path.basename(dst))

    best = os.path.join(run_dir, "weights", "best.pt")
    print(f"\nbest weights : {best}")
    print(f"curves       : {fig_png}")
    if copied:
        print("also copied  : " + ", ".join(copied))
    print("\noriginal weights untouched:", a.weights)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
