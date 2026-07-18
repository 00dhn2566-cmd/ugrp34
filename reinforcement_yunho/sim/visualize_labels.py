"""Visualise a YOLO-pose window label over its image to eyeball corner order.

WHAT
----
Given an image and its YOLO-pose ``.txt`` (same stem), draw for each labelled
window: the axis-aligned bbox and the 4 keypoints, each coloured + NUMBERED in
CORNER_ORDER (0 top_left, 1 top_right, 2 bottom_right, 3 bottom_left). 길남 uses
this to confirm corner ordering and normalisation before training.

WHY THREE BACKENDS
------------------
Rendering falls back gracefully so it ALWAYS produces output:
    matplotlib  (--backend mpl)  -> nicest, needs matplotlib
    PIL/Pillow  (--backend pil)  -> needs Pillow
    SVG         (--backend svg)  -> pure Python (stdlib only), never fails
Default --backend auto tries mpl, then pil, then svg.

Denormalisation needs the image size. mpl/pil read it from the image; svg reads
it from --width/--height (or, if the image is a PNG, from its header).
"""
from __future__ import annotations

import argparse
import base64
import os
import struct
import sys
from typing import Dict, List, Optional, Tuple

# --- shared 'common' for CORNER_ORDER ----------------------------------------
_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)
from common import CORNER_ORDER  # noqa: E402

# Colour per keypoint index so ordering is visible at a glance (index -> hex).
#   0 top_left=red, 1 top_right=green, 2 bottom_right=blue, 3 bottom_left=yellow
CORNER_COLORS = ("#e6194b", "#3cb44b", "#4363d8", "#ffe119")
# Bbox stroke colour per class (== traversal order_index; CONVENTIONS.md).
CLASS_COLORS = {0: "#e6194b", 1: "#3cb44b", 2: "#4363d8"}


# ======================================================================
#  PURE PARSING  (stdlib only)
# ======================================================================
def parse_label_line(line: str) -> Dict[str, object]:
    """Parse one YOLO-pose line -> normalised {cls, bbox(cx,cy,w,h), kpts[(u,v,vis)]}.

    Raises ValueError on the wrong token count (expect 17 = 1 + 4 + 4*3).
    """
    tok = line.split()
    if len(tok) != 17:
        raise ValueError(f"expected 17 tokens (1+4+4*3), got {len(tok)}: {line!r}")
    cls = int(float(tok[0]))
    cx, cy, w, h = (float(t) for t in tok[1:5])
    kpts: List[Tuple[float, float, float]] = []
    for i in range(4):
        u, v, vis = (float(t) for t in tok[5 + 3 * i : 8 + 3 * i])
        kpts.append((u, v, vis))
    return {"cls": cls, "bbox": (cx, cy, w, h), "kpts": kpts}


def read_labels(txt_path: str) -> List[Dict[str, object]]:
    with open(txt_path, "r") as f:
        return [parse_label_line(ln) for ln in f if ln.strip()]


def _png_size(path: str) -> Optional[Tuple[int, int]]:
    """Read (width,height) from a PNG header without any image library."""
    try:
        with open(path, "rb") as f:
            head = f.read(24)
        if len(head) >= 24 and head[:8] == b"\x89PNG\r\n\x1a\n" and head[12:16] == b"IHDR":
            w, h = struct.unpack(">II", head[16:24])
            return int(w), int(h)
    except Exception:
        pass
    return None


# ======================================================================
#  SVG BACKEND  (pure python -- always available)
# ======================================================================
def build_svg(
    labels: List[Dict[str, object]],
    width: int,
    height: int,
    image_path: Optional[str] = None,
) -> str:
    """Render labels to an SVG string. Embeds the image (base64) if it exists."""
    parts: List[str] = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" '
        f'viewBox="0 0 {width} {height}">'
    ]
    # background: embed the real PNG if we can, else a neutral grey canvas
    embedded = False
    if image_path and os.path.exists(image_path) and image_path.lower().endswith(".png"):
        with open(image_path, "rb") as f:
            b64 = base64.b64encode(f.read()).decode()
        parts.append(
            f'<image href="data:image/png;base64,{b64}" x="0" y="0" '
            f'width="{width}" height="{height}"/>'
        )
        embedded = True
    if not embedded:
        parts.append(f'<rect x="0" y="0" width="{width}" height="{height}" fill="#303030"/>')

    r = max(3.0, min(width, height) * 0.008)  # keypoint radius, scale-aware
    for lab in labels:
        cls = int(lab["cls"])
        cx, cy, bw, bh = lab["bbox"]  # normalised
        x = (cx - bw / 2.0) * width
        y = (cy - bh / 2.0) * height
        stroke = CLASS_COLORS.get(cls, "#ffffff")
        parts.append(
            f'<rect x="{x:.2f}" y="{y:.2f}" width="{bw * width:.2f}" '
            f'height="{bh * height:.2f}" fill="none" stroke="{stroke}" '
            f'stroke-width="2"/>'
        )
        for i, (u, v, vis) in enumerate(lab["kpts"]):
            px, py = u * width, v * height
            col = CORNER_COLORS[i]
            op = "1.0" if vis >= 0.5 else "0.35"  # dim vis=0 corners (dataset 2)
            parts.append(
                f'<circle cx="{px:.2f}" cy="{py:.2f}" r="{r:.2f}" fill="{col}" '
                f'stroke="#000000" stroke-width="1" fill-opacity="{op}"/>'
            )
            parts.append(
                f'<text x="{px + r + 2:.2f}" y="{py - r:.2f}" font-size="{r * 2.2:.1f}" '
                f'fill="{col}" stroke="#000000" stroke-width="0.4">'
                f'{i}:{CORNER_ORDER[i]}</text>'
            )
    parts.append("</svg>")
    return "\n".join(parts)


# ======================================================================
#  OPTIONAL RICH BACKENDS  (import-guarded)
# ======================================================================
def _render_mpl(labels, width, height, image_path, out_path) -> bool:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.patches as patches
        import matplotlib.pyplot as plt
    except Exception:
        return False
    fig, ax = plt.subplots(figsize=(width / 100.0, height / 100.0))
    if image_path and os.path.exists(image_path):
        ax.imshow(plt.imread(image_path))
    else:
        ax.add_patch(patches.Rectangle((0, 0), width, height, color="#303030"))
    for lab in labels:
        cls = int(lab["cls"])
        cx, cy, bw, bh = lab["bbox"]
        ax.add_patch(
            patches.Rectangle(
                ((cx - bw / 2) * width, (cy - bh / 2) * height),
                bw * width, bh * height, fill=False,
                edgecolor=CLASS_COLORS.get(cls, "w"), linewidth=2,
            )
        )
        for i, (u, v, vis) in enumerate(lab["kpts"]):
            ax.scatter([u * width], [v * height], c=CORNER_COLORS[i],
                       s=60, edgecolors="k", alpha=1.0 if vis >= 0.5 else 0.35)
            ax.text(u * width + 4, v * height - 4, f"{i}:{CORNER_ORDER[i]}",
                    color=CORNER_COLORS[i], fontsize=8)
    ax.set_xlim(0, width)
    ax.set_ylim(height, 0)
    ax.axis("off")
    fig.savefig(out_path, bbox_inches="tight", dpi=100)
    plt.close(fig)
    return True


def _render_pil(labels, width, height, image_path, out_path) -> bool:
    try:
        from PIL import Image, ImageDraw
    except Exception:
        return False
    if image_path and os.path.exists(image_path):
        img = Image.open(image_path).convert("RGB")
        width, height = img.size
    else:
        img = Image.new("RGB", (width, height), (48, 48, 48))
    draw = ImageDraw.Draw(img)
    for lab in labels:
        cls = int(lab["cls"])
        cx, cy, bw, bh = lab["bbox"]
        x0, y0 = (cx - bw / 2) * width, (cy - bh / 2) * height
        x1, y1 = (cx + bw / 2) * width, (cy + bh / 2) * height
        draw.rectangle([x0, y0, x1, y1], outline=CLASS_COLORS.get(cls, "#ffffff"), width=2)
        for i, (u, v, vis) in enumerate(lab["kpts"]):
            px, py = u * width, v * height
            r = max(3, min(width, height) // 120)
            draw.ellipse([px - r, py - r, px + r, py + r], fill=CORNER_COLORS[i], outline="black")
            draw.text((px + r + 2, py - r), f"{i}:{CORNER_ORDER[i]}", fill=CORNER_COLORS[i])
    img.save(out_path)
    return True


# ======================================================================
#  DISPATCH + CLI
# ======================================================================
def render(
    labels: List[Dict[str, object]],
    out_path: str,
    *,
    backend: str = "auto",
    image_path: Optional[str] = None,
    width: Optional[int] = None,
    height: Optional[int] = None,
) -> str:
    """Render labels to ``out_path``; returns the backend actually used."""
    # resolve image size for backends that need it up front (svg)
    if (width is None or height is None) and image_path:
        wh = _png_size(image_path)
        if wh:
            width, height = wh

    if backend in ("auto", "mpl") and _render_mpl(labels, width or 1280, height or 720, image_path, out_path):
        return "mpl"
    if backend in ("auto", "pil") and _render_pil(labels, width or 1280, height or 720, image_path, out_path):
        return "pil"
    # svg fallback (always works); needs a size
    if width is None or height is None:
        raise SystemExit("SVG backend needs --width/--height (image size unknown).")
    svg = build_svg(labels, width, height, image_path=image_path)
    if not out_path.lower().endswith(".svg"):
        out_path = os.path.splitext(out_path)[0] + ".svg"
    with open(out_path, "w") as f:
        f.write(svg)
    return "svg"


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--label", required=True, help="YOLO-pose .txt label file.")
    p.add_argument("--image", help="image file (optional; drawn as background if present).")
    p.add_argument("--out", required=True, help="output image/SVG path.")
    p.add_argument("--backend", choices=("auto", "mpl", "pil", "svg"), default="auto")
    p.add_argument("--width", type=int, help="image width (px); needed for svg without a PNG.")
    p.add_argument("--height", type=int, help="image height (px); needed for svg without a PNG.")
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    labels = read_labels(args.label)
    used = render(
        labels, args.out, backend=args.backend,
        image_path=args.image, width=args.width, height=args.height,
    )
    print(f"rendered {len(labels)} window(s) with backend={used}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
