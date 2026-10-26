"""One-episode-per-env evaluation on a fixed, seeded scene set (README §7.4).

Every planner sees the identical scenes and identical camera-noise draws: before
each planner the caller re-seeds the core (``core.reseed``) and resets all envs.
Camera noise is drawn with a fixed shape every decision from its own stream, and
the draws whose count depends on the planner (resets after an episode ends,
observation noise) come from separate streams, so each env's first episode is
the same experiment for every planner. ``scene_fingerprint`` lets callers check it.
"""
from __future__ import annotations

import csv
import json
import os
from typing import Dict, List, Mapping

import torch

from .core.mission import CRASH_NAMES

RECORD_KEYS = ("success", "crashed", "crash_reason", "windows_passed", "n_windows", "time_s",
               "clearance_mean", "clearance_min", "level", "est_err_at_pass", "return")


@torch.no_grad()
def run_planner(env, planner, max_steps: int | None = None, record_k: int = 0) -> Dict[str, torch.Tensor]:
    """Run one episode per env. ``record_k`` > 0 also stores the first k envs' trajectories
    (positions per decision until their first episode ends) and their scenes in out["traj"]."""
    core = env.core
    E = env.num_envs
    finished = torch.zeros(E, dtype=torch.bool, device=env.device)
    out = {k: torch.zeros_like(v) for k, v in core.episode_record.items()}
    planner.reset(torch.arange(E, device=env.device))
    steps = max_steps or core.max_episode_length + 5
    nrec = min(record_k, E)
    traj = [[] for _ in range(nrec)]
    scene_k = None
    if nrec:
        sc = core.scene
        scene_k = {"center": sc.center[:nrec].cpu(), "normal": sc.normal[:nrec].cpu(), "size": sc.size[:nrec].cpu(),
                   "n_windows": sc.n_windows[:nrec].cpu(), "start": sc.start[:nrec].cpu()}
    for _ in range(steps):
        if nrec:
            pos = core.state.pos[:nrec].detach().cpu()
            for i in range(nrec):
                if not bool(finished[i]):
                    traj[i].append(pos[i].tolist())
        act = planner.act(core)
        _, _, dones, _ = env.step(act)
        d = dones.bool()
        if d.any():
            ids = d.nonzero(as_tuple=False).squeeze(-1)
            planner.reset(ids)
            new = d & ~finished
            for k, v in core.episode_record.items():
                out[k] = torch.where(new.view(-1, *([1] * (v.dim() - 1))), v, out[k])
            finished |= d
        if bool(finished.all()):
            break
    out["finished"] = finished
    if nrec:
        out["traj"] = {"paths": traj, "scene": scene_k}
    return out


def scene_fingerprint(scene) -> str:
    """Short hash of the scene tensors (to assert that planners were compared on the same scenes)."""
    import hashlib
    h = hashlib.sha1()
    for t in (scene.center, scene.normal, scene.size, scene.n_windows, scene.start):
        h.update(t.detach().float().cpu().numpy().tobytes())
    return h.hexdigest()[:12]


def summarize(rec: Mapping[str, torch.Tensor]) -> dict:
    f = rec["finished"]
    n = int(f.sum())
    s = lambda t: float(t[f].float().mean()) if n else float("nan")  # noqa: E731
    succ = rec["success"] & f
    out = {
        "episodes": n,
        "success_rate": s(rec["success"]),
        "crash_rate": s(rec["crashed"]),
        "timeout_rate": float((f & ~rec["success"] & ~rec["crashed"]).float().sum() / max(n, 1)),
        "windows_passed_frac": float((rec["windows_passed"][f].float() / rec["n_windows"][f].float().clamp_min(1)).mean()) if n else float("nan"),
        "mean_time_success_s": float(rec["time_s"][succ].mean()) if succ.any() else float("nan"),
        "mean_clearance_m": float(rec["clearance_mean"][f & (rec["windows_passed"] > 0)].mean()) if (f & (rec["windows_passed"] > 0)).any() else float("nan"),
        "min_clearance_m": float(rec["clearance_min"][f & (rec["windows_passed"] > 0)].min()) if (f & (rec["windows_passed"] > 0)).any() else float("nan"),
        "est_err_at_pass_m": float(rec["est_err_at_pass"][f & (rec["windows_passed"] > 0)].mean()) if (f & (rec["windows_passed"] > 0)).any() else float("nan"),
        "mean_return": s(rec["return"]),
    }
    for code, name in enumerate(CRASH_NAMES[1:], start=1):
        out[f"crash_{name}"] = float(((rec["crash_reason"] == code) & rec["crashed"] & f).float().sum() / max(n, 1))
    return out


def write_csv(path: str, rows: List[dict]) -> None:
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    keys = list(rows[0].keys())
    with open(path, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)


def per_env_rows(planner_name: str, rec: Mapping[str, torch.Tensor]) -> List[dict]:
    rows = []
    E = rec["finished"].shape[0]
    cpu = {k: v.detach().cpu() for k, v in rec.items() if k != "traj"}
    for e in range(E):
        r = {"planner": planner_name, "env": e}
        for k in RECORD_KEYS + ("finished",):
            v = cpu[k][e]
            r[k] = (CRASH_NAMES[int(v)] if k == "crash_reason" else (v.item() if v.dim() == 0 else v.tolist()))
        rows.append(r)
    return rows


def save_json(path: str, obj) -> None:
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(obj, fh, indent=2, ensure_ascii=False)
