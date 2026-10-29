"""Onboard-camera frame annotation for the vision-in-the-loop GIFs (scripts/vision_loop.py): the detector's corners and
class on the rendered image, the true corners of the windows the surrogate would see, and a status bar."""
from __future__ import annotations

import os

from PIL import Image, ImageDraw, ImageFont

PAL = [(235, 64, 52), (52, 199, 89), (64, 140, 255)]          # frame colour classes: red, green, blue
NAMES = ("red", "green", "blue")
INK, DIM, BAR = (240, 244, 250), (160, 170, 185), (12, 16, 26)
FONTS = ("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf")
_cache: dict = {}


def font(size: int, bold: bool = False):
    key = (size, bold)
    if key not in _cache:
        path = FONTS[bold]
        _cache[key] = ImageFont.truetype(path, size) if os.path.exists(path) else ImageFont.load_default(size)
    return _cache[key]


def annotate(img, dets: list, truth: list, status: str, title: str = "onboard camera  ·  YOLO-pose corners") -> Image.Image:
    """img (H,W,3) uint8 RGB; dets = YoloCorners.detect output; truth = true corner pixels [(4,2)] of the windows the
    surrogate would detect from this pose; status = the right-hand text of the top bar. -> RGB image, same size."""
    im = Image.fromarray(img).convert("RGBA")
    over = Image.new("RGBA", im.size, (0, 0, 0, 0))
    dr = ImageDraw.Draw(over)
    W = im.size[0]
    for cls, cf, kp, _ in dets:
        pts = [tuple(map(float, q)) for q in kp]
        dr.line(pts + [pts[0]], fill=(0, 0, 0, 150), width=7)
        dr.line(pts + [pts[0]], fill=INK + (255,), width=3)
        c = PAL[cls % 3]
        for x, y in pts:
            dr.ellipse((x - 9, y - 9, x + 9, y + 9), fill=c + (255,), outline=INK + (255,), width=2)
        tag = f"{NAMES[cls % 3]} {cf:.2f}"
        f = font(22, True)
        x0, y0 = min(p[0] for p in pts), min(p[1] for p in pts) - 40
        x0, y0 = max(4.0, min(x0, W - 8 - dr.textlength(tag, font=f) - 16)), max(64.0, y0)
        dr.rounded_rectangle((x0, y0, x0 + dr.textlength(tag, font=f) + 16, y0 + 32), radius=6, fill=c + (230,))
        dr.text((x0 + 8, y0 + 3), tag, font=f, fill=INK + (255,))
    for q in truth:
        for x, y in (tuple(map(float, p)) for p in q):
            dr.line((x - 6, y, x + 6, y), fill=(0, 0, 0, 255), width=4)
            dr.line((x, y - 6, x, y + 6), fill=(0, 0, 0, 255), width=4)
            dr.line((x - 5, y, x + 5, y), fill=(255, 255, 255, 255), width=2)
            dr.line((x, y - 5, x, y + 5), fill=(255, 255, 255, 255), width=2)
    dr.rectangle((0, 0, W, 54), fill=BAR + (215,))
    dr.text((18, 13), title, font=font(24, True), fill=INK + (255,))
    f = font(24)
    dr.text((W - 18 - dr.textlength(status, font=f), 13), status, font=f, fill=INK + (255,))
    leg = "●  detector corners      +  true corners"
    dr.rounded_rectangle((12, im.size[1] - 48, 24 + dr.textlength(leg, font=font(20)), im.size[1] - 12), radius=6,
                         fill=BAR + (190,))
    dr.text((18, im.size[1] - 44), leg, font=font(20), fill=DIM + (255,))
    return Image.alpha_composite(im, over).convert("RGB")
