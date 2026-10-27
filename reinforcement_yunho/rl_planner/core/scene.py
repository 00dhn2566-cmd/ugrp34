"""Vectorised window-layout sampler.

Every env holds up to ``max_windows`` windows; unused slots are flagged invalid
and parked far away so no geometry test can ever touch them.

Window model (CONVENTIONS.md "Window model", common/geometry.py):
  local +X = right, +Y = up, +Z = normal pointing to the APPROACH side.
  corners in CORNER_ORDER: 0 TL, 1 TR, 2 BR, 3 BL (clockwise seen from the front).
  normal == normalize(cross(BL - TL, TR - TL))  (state_window spec §3.1 confirmed sign)

The drone flies toward +x, so a window with yaw 0 has normal (-1, 0, 0).
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping, Sequence

import torch

from . import math3d as m3
from .config import level_params, num_levels

PARK_X = 1.0e4   # where invalid window slots live


@dataclass
class SceneTensors:
    n_windows: torch.Tensor   # (E,) long
    valid: torch.Tensor       # (E, W) bool
    center: torch.Tensor      # (E, W, 3)
    normal: torch.Tensor      # (E, W, 3) unit, approach side
    right: torch.Tensor       # (E, W, 3) unit
    up: torch.Tensor          # (E, W, 3) unit
    size: torch.Tensor        # (E, W, 2) opening (w, h)
    corners: torch.Tensor     # (E, W, 4, 3) world, CORNER_ORDER
    color: torch.Tensor       # (E, W) long, 0 red / 1 green / 2 blue (= order_index % 3)
    start: torch.Tensor       # (E, 3)

    @staticmethod
    def empty(num_envs: int, max_windows: int, device) -> "SceneTensors":
        E, W = num_envs, max_windows
        z = lambda *s: torch.zeros(*s, device=device)  # noqa: E731
        return SceneTensors(
            n_windows=torch.zeros(E, dtype=torch.long, device=device),
            valid=torch.zeros(E, W, dtype=torch.bool, device=device),
            center=z(E, W, 3), normal=z(E, W, 3), right=z(E, W, 3), up=z(E, W, 3),
            size=z(E, W, 2), corners=z(E, W, 4, 3),
            color=torch.zeros(E, W, dtype=torch.long, device=device), start=z(E, 3))


def window_frame(normal: torch.Tensor):
    """normal (...,3) -> (right, up) with right = cross(UP, n), up = cross(n, right)."""
    up_w = torch.zeros_like(normal)
    up_w[..., 2] = 1.0
    right = m3.normalize(torch.cross(up_w, normal, dim=-1))
    up = m3.normalize(torch.cross(normal, right, dim=-1))
    return right, up


def window_corners(center, right, up, size) -> torch.Tensor:
    """(...,3)x3 + (...,2) -> (...,4,3) in CORNER_ORDER (TL, TR, BR, BL)."""
    hw = (size[..., 0:1] / 2).unsqueeze(-2)
    hh = (size[..., 1:2] / 2).unsqueeze(-2)
    sx = torch.tensor([-1.0, 1.0, 1.0, -1.0], device=center.device).view(*([1] * (center.dim() - 1)), 4, 1)
    sy = torch.tensor([1.0, 1.0, -1.0, -1.0], device=center.device).view(*([1] * (center.dim() - 1)), 4, 1)
    return center.unsqueeze(-2) + sx * hw * right.unsqueeze(-2) + sy * hh * up.unsqueeze(-2)


class LevelTable:
    """Per-curriculum-level sampling ranges as tensors: param -> (L, 2)."""

    KEYS = ("n_windows", "first_gap", "spacing", "lateral", "height", "yaw_deg", "turn_deg", "rise",
            "opening_w", "opening_h", "corner_noise_px", "miss_prob")

    def __init__(self, cfg: Mapping, device):
        self.num_levels = num_levels(cfg)
        rows = [level_params(cfg, lv) for lv in range(self.num_levels)]
        self.t = {k: torch.tensor([[float(r[k][0]), float(r[k][1])] for r in rows],
                                  device=device) for k in self.KEYS}

    def uniform(self, key: str, level: torch.Tensor, gen: torch.Generator,
                shape: Sequence[int] = ()) -> torch.Tensor:
        lohi = self.t[key][level]                       # (E, 2)
        lo, hi = lohi[:, 0], lohi[:, 1]
        for _ in shape:
            lo, hi = lo.unsqueeze(-1), hi.unsqueeze(-1)
        u = torch.rand((level.shape[0], *shape), generator=gen, device=level.device)
        return lo + (hi - lo) * u

    def randint(self, key: str, level: torch.Tensor, gen: torch.Generator) -> torch.Tensor:
        lohi = self.t[key][level].round().long()
        span = (lohi[:, 1] - lohi[:, 0] + 1).clamp_min(1)
        u = torch.rand(level.shape[0], generator=gen, device=level.device)
        return lohi[:, 0] + (u * span).floor().long().clamp_max(span - 1)


def _seg_dist(p0: torch.Tensor, p1: torch.Tensor, q: torch.Tensor) -> torch.Tensor:
    """Distance from points ``q`` to segments ``p0 -> p1`` (all (..., 3), broadcast)."""
    d = p1 - p0
    t = (((q - p0) * d).sum(-1) / (d * d).sum(-1).clamp_min(1e-9)).clamp(0.0, 1.0)
    return (p0 + t[..., None] * d - q).norm(dim=-1)


def sample_scatter(cfg: Mapping, table: LevelTable, lv: torch.Tensor, gen: torch.Generator, start: torch.Tensor,
                   n_win: torch.Tensor, W: int):
    """Scatter course (scene.layout = scatter): every window is placed from the one before it (the start for the
    first) at distance ``spacing`` (``first_gap`` for the first), bearing ±``turn_deg`` off the direction the drone
    leaves that window in (sign random) and height change ``rise`` (clipped to ``height``); its normal faces back
    along that bearing, tilted by ``yaw_deg``. So right after a pass the next window can be far to the side, above
    or below the camera's view.

    The order is the "nearest exit" rule: from the start and from every window, the next window is the nearest of
    all windows not passed yet, every other one at least ``nearest_margin`` times farther. Window centres keep
    ``min_separation``; the straight legs (and 1 m past each window) stay ``path_clearance`` outside every other wall
    panel (a panel = disc of radius wall_half_extent * sqrt 2 around its centre). Each window takes the first of
    ``place_tries`` candidates that satisfies everything, preferring legs that point down the arena (+x within 90 deg)
    so long courses do not curl back into themselves; when none fits the course ends there.

    Returns (center (n,W,3), pass heading (n,W) [rad], number of windows placed (n,))."""
    sc = cfg["scene"]
    device = start.device
    n_e = start.shape[0]
    K = max(1, int(sc.get("place_tries", 32)))
    margin, sep = float(sc.get("nearest_margin", 1.25)), float(sc.get("min_separation", 2.4))
    r_clear = float(sc["wall_half_extent"]) * 2 ** 0.5 + float(sc.get("path_clearance", 0.4))
    b = sc["bounds"]
    x_lo, y_hi = float(b["x_min"]) + 1.5, float(b["y_abs"]) - 1.5
    # every draw up front with a fixed shape: the stream does not depend on which candidates succeed
    dist = table.uniform("spacing", lv, gen, (W, K))
    dist[:, 0] = table.uniform("first_gap", lv, gen, (K,))
    sgn = torch.where(torch.rand(n_e, W, K, generator=gen, device=device) < 0.5, -1.0, 1.0)
    turn = torch.deg2rad(table.uniform("turn_deg", lv, gen, (W, K))) * sgn
    rise = table.uniform("rise", lv, gen, (W, K))
    tilt = torch.deg2rad(table.uniform("yaw_deg", lv, gen, (W, K)))
    z_lo, z_hi = table.t["height"][lv, 0][:, None], table.t["height"][lv, 1][:, None]

    center = torch.zeros(n_e, W, 3, device=device)
    heading = torch.zeros(n_e, W, device=device)
    placed = torch.zeros(n_e, dtype=torch.long, device=device)
    alive = n_win > 0
    ref, head = start.clone(), torch.zeros(n_e, device=device)
    ar = torch.arange(n_e, device=device)
    for k in range(W):
        alive = alive & (k < n_win)
        if not bool(alive.any()):
            break
        phi = head[:, None] + turn[:, k]                                       # (n,K) bearing of the leg
        cand = torch.stack((ref[:, None, 0] + dist[:, k] * torch.cos(phi), ref[:, None, 1] + dist[:, k] * torch.sin(phi),
                            torch.maximum(torch.minimum(ref[:, None, 2] + rise[:, k], z_hi), z_lo)), -1)
        ok = (cand[..., 0] >= x_lo) & (cand[..., 1].abs() <= y_hi)
        over = ref + torch.stack((torch.cos(head), torch.sin(head), torch.zeros_like(head)), -1)   # 1 m past the window
        if k > 0:
            prev = center[:, :k]                                               # (n,k,3)
            ok &= ((cand[:, :, None] - prev[:, None]).norm(dim=-1) >= sep).all(-1)
            refs = torch.cat((start[:, None], prev[:, :-1]), 1)                # R_j: where window j is chosen from
            d_next = (prev - refs).norm(dim=-1)                                # distance from R_j to window j
            ok &= ((cand[:, :, None] - refs[:, None]).norm(dim=-1) >= margin * d_next[:, None]).all(-1)
            # the new window's panel stays off every earlier leg (R_j -> window j)
            ok &= (_seg_dist(refs[:, None], prev[:, None], cand[:, :, None]) >= r_clear).all(-1)
        if k > 1:                                                              # the new leg stays off earlier panels
            obst = center[:, :k - 1][:, None]                                  # (n,1,k-1,3), not the window just left
            for a, c in ((ref[:, None, None], cand[:, :, None]), (ref[:, None, None], over[:, None, None]),
                         (over[:, None, None], cand[:, :, None])):
                ok &= (_seg_dist(a, c, obst) >= r_clear).all(-1)
        fwd = torch.cos(phi) >= 0.0                                            # leg points down the arena
        pick = (ok.float() * (1.0 + fwd.float())).argmax(-1)                   # first fitting one, forward first
        go = alive & ok.any(-1)
        alive = go
        c = cand[ar, pick]
        center[:, k] = torch.where(go[:, None], c, center[:, k])
        hd = phi[ar, pick] + tilt[ar, k, pick]
        heading[:, k] = torch.where(go, hd, heading[:, k])
        placed = placed + go.long()
        ref = torch.where(go[:, None], c, ref)
        head = torch.where(go, hd, head)
    return center, heading, placed


def sample_scene(cfg: Mapping, table: LevelTable, level: torch.Tensor, gen: torch.Generator,
                 out: SceneTensors, env_ids: torch.Tensor) -> None:
    """Resample layouts for ``env_ids`` in place (levels given per env in ``level``)."""
    if env_ids.numel() == 0:
        return
    device = level.device
    W = out.center.shape[1]
    lv = level[env_ids]
    n_e = env_ids.numel()

    n_win = table.randint("n_windows", lv, gen).clamp(1, W)

    d = cfg["drone"]
    start = torch.tensor(d["start_pos"], device=device).expand(n_e, 3).clone()
    jitter = torch.tensor(d["start_pos_noise"], device=device)
    start += (torch.rand(n_e, 3, generator=gen, device=device) * 2 - 1) * jitter

    if cfg["scene"].get("layout", "line") == "scatter":
        center, yaw, placed = sample_scatter(cfg, table, lv, gen, start, n_win, W)
        for _ in range(int(cfg["scene"].get("place_retries", 8))):     # redraw courses that ended short
            short = (placed < n_win).nonzero(as_tuple=False).squeeze(-1)
            if short.numel() == 0:
                break
            c2, y2, p2 = sample_scatter(cfg, table, lv[short], gen, start[short], n_win[short], W)
            better = short[p2 > placed[short]]
            take = p2 > placed[short]
            center[better], yaw[better], placed[better] = c2[take], y2[take], p2[take]
        n_win = placed
        x, y, z = center.unbind(-1)
    else:
        gaps = table.uniform("spacing", lv, gen, (W,))
        gaps[:, 0] = table.uniform("first_gap", lv, gen)
        x = start[:, 0:1] + torch.cumsum(gaps, dim=1)
        y = table.uniform("lateral", lv, gen, (W,))
        z = table.uniform("height", lv, gen, (W,))
        yaw = torch.deg2rad(table.uniform("yaw_deg", lv, gen, (W,)))
    ow = table.uniform("opening_w", lv, gen, (W,))
    oh = table.uniform("opening_h", lv, gen, (W,))
    slot = torch.arange(W, device=device).unsqueeze(0).expand(n_e, W)
    valid = slot < n_win.unsqueeze(1)

    x = torch.where(valid, x, torch.full_like(x, PARK_X))
    center = torch.stack((x, y, z), -1)
    normal = torch.stack((-torch.cos(yaw), -torch.sin(yaw), torch.zeros_like(yaw)), -1)
    right, up = window_frame(normal)
    size = torch.stack((ow, oh), -1)

    out.n_windows[env_ids] = n_win
    out.valid[env_ids] = valid
    out.center[env_ids] = center
    out.normal[env_ids] = normal
    out.right[env_ids] = right
    out.up[env_ids] = up
    out.size[env_ids] = size
    out.corners[env_ids] = window_corners(center, right, up, size)
    out.color[env_ids] = slot % 3
    out.start[env_ids] = start


def scene_from_layout(layouts: Sequence[Sequence[Mapping]], max_windows: int, start, device) -> SceneTensors:
    """Build SceneTensors from explicit layouts (prototype_demo style dicts:
    center, ow/oh or size_wh, optional normal). Used by evaluation / integration."""
    E = len(layouts)
    sc = SceneTensors.empty(E, max_windows, device)
    for e, lay in enumerate(layouts):
        for i, w in enumerate(lay[:max_windows]):
            c = torch.tensor(w["center"], dtype=torch.float32, device=device)
            n = torch.tensor(w.get("normal", (-1.0, 0.0, 0.0)), dtype=torch.float32, device=device)
            if "size_wh" in w:
                s = torch.tensor(w["size_wh"], dtype=torch.float32, device=device)
            else:
                s = torch.tensor([w["ow"], w["oh"]], dtype=torch.float32, device=device)
            sc.center[e, i], sc.normal[e, i], sc.size[e, i] = c, m3.normalize(n), s
            sc.valid[e, i] = True
        sc.n_windows[e] = min(len(lay), max_windows)
    sc.center[~sc.valid] = torch.tensor([PARK_X, 0.0, 1.0], device=device)
    sc.normal[~sc.valid] = torch.tensor([-1.0, 0.0, 0.0], device=device)
    sc.size[~sc.valid] = 1.0
    sc.right, sc.up = window_frame(sc.normal)
    sc.corners = window_corners(sc.center, sc.right, sc.up, sc.size)
    sc.color = torch.arange(max_windows, device=device).unsqueeze(0).expand(E, -1) % 3
    sc.start = torch.as_tensor(start, dtype=torch.float32, device=device).expand(E, 3).clone()
    return sc
