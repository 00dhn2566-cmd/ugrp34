"""Config loading: default.yaml <- experiment yamls <- ``a.b.c=value`` overrides.

The merged config is a plain nested dict wrapped in :class:`Cfg` for dot access.
It is deliberately framework-free (no Isaac Lab ``configclass``) so the same file
drives the torch backend, the Isaac Lab backend, the FSM baseline and evaluation.
"""
from __future__ import annotations

import copy
import hashlib
import json
import os
from typing import Any, Iterable, Mapping

import yaml

CONFIG_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "configs")
DEFAULT_CONFIG = os.path.join(CONFIG_DIR, "default.yaml")


class Cfg(dict):
    """dict with attribute access; nested dicts are wrapped on access."""

    def __getattr__(self, key: str) -> Any:
        try:
            v = self[key]
        except KeyError as e:
            raise AttributeError(key) from e
        return Cfg(v) if isinstance(v, dict) and not isinstance(v, Cfg) else v

    def __setattr__(self, key: str, value: Any) -> None:
        self[key] = value

    def to_dict(self) -> dict:
        return json.loads(json.dumps(self))


def deep_merge(base: dict, over: Mapping) -> dict:
    out = copy.deepcopy(base)
    for k, v in over.items():
        if isinstance(v, Mapping) and isinstance(out.get(k), dict):
            out[k] = deep_merge(out[k], v)
        else:
            out[k] = copy.deepcopy(v)
    return out


def apply_override(cfg: dict, dotted: str) -> None:
    """``"reward.crash=-30"`` -> cfg["reward"]["crash"] = -30 (value parsed as YAML)."""
    if "=" not in dotted:
        raise ValueError(f"override must look like key.path=value, got {dotted!r}")
    path, raw = dotted.split("=", 1)
    keys = path.strip().split(".")
    node = cfg
    for k in keys[:-1]:
        if k not in node or not isinstance(node[k], dict):
            raise KeyError(f"override path {path!r}: {k!r} is not a config section")
        node = node[k]
    if keys[-1] not in node:
        raise KeyError(f"override path {path!r}: unknown key {keys[-1]!r} "
                       f"(add it to default.yaml first so every run records it)")
    node[keys[-1]] = yaml.safe_load(raw)


def load_config(extra: Iterable[str] = (), overrides: Iterable[str] = ()) -> Cfg:
    with open(DEFAULT_CONFIG, encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    for path in extra:
        with open(path, encoding="utf-8") as f:
            cfg = deep_merge(cfg, yaml.safe_load(f) or {})
    for o in overrides:
        apply_override(cfg, o)
    return Cfg(cfg)


# training-time augmentations: an evaluation falls back to default.yaml for these, so policies
# trained with different augmentations are compared on the same distribution
TRAIN_ONLY_KEYS = (("domain_randomization", "blackout_prob"), ("curriculum", "fixed_level"))
# runtime-only sections (not part of the MDP): always today's default.yaml, never the training snapshot
RUNTIME_SECTIONS = ("safety",)


def default_section(name: str) -> dict:
    with open(DEFAULT_CONFIG, encoding="utf-8") as f:
        return copy.deepcopy(yaml.safe_load(f).get(name) or {})


def config_for_policy(trained: Mapping, extra: Iterable[str] = (), overrides: Iterable[str] = ()) -> Cfg:
    """Evaluation config for a trained policy: default.yaml <- the policy's training config (so the
    MDP matches; keys added to default.yaml later keep their defaults) <- extra yamls <- overrides.
    TRAIN_ONLY_KEYS and RUNTIME_SECTIONS revert to default.yaml before the extras, which can still
    set them."""
    with open(DEFAULT_CONFIG, encoding="utf-8") as f:
        default = yaml.safe_load(f)
    cfg = deep_merge(default, trained)
    for sec, key in TRAIN_ONLY_KEYS:
        if key in default.get(sec, {}):
            cfg[sec][key] = copy.deepcopy(default[sec][key])
    for sec in RUNTIME_SECTIONS:
        if sec in default:
            cfg[sec] = copy.deepcopy(default[sec])
    for path in extra:
        with open(path, encoding="utf-8") as f:
            cfg = deep_merge(cfg, yaml.safe_load(f) or {})
    for o in overrides:
        apply_override(cfg, o)
    return Cfg(cfg)


def config_hash(cfg: Mapping) -> str:
    blob = json.dumps(cfg, sort_keys=True, default=str).encode()
    return hashlib.sha1(blob).hexdigest()[:10]


def save_config(cfg: Mapping, path: str) -> None:
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        yaml.safe_dump(json.loads(json.dumps(cfg)), f, allow_unicode=True, sort_keys=False)


def level_params(cfg: Mapping, level: int) -> dict:
    """Merge curriculum level ``level`` over the base scene/perception ranges.

    Returns a flat dict of the per-level sampling ranges used by the scene sampler
    and the perception model (keys: n_windows, spacing, lateral, height, yaw_deg,
    opening_w, opening_h, first_gap, corner_noise_px, miss_prob).
    """
    scene, perc = cfg["scene"], cfg["perception"]
    base = {
        "n_windows": [scene["max_windows"], scene["max_windows"]],
        "first_gap": scene["first_gap"],
        "spacing": scene["spacing"],
        "lateral": scene["lateral"],
        "height": scene["height"],
        "yaw_deg": scene["yaw_deg"],
        "opening_w": scene["opening_w"],
        "opening_h": scene["opening_h"],
        "corner_noise_px": perc["corner_noise_px"],
        "miss_prob": perc["miss_prob"],
    }
    cur = cfg["curriculum"]
    levels = cur["levels"] if cur.get("enabled", True) else []
    if levels:
        lv = levels[max(0, min(int(level), len(levels) - 1))]
        for k, v in lv.items():
            if k not in base:
                raise KeyError(f"curriculum level key {k!r} is not a level parameter "
                               f"(allowed: {sorted(base)})")
            base[k] = v
    return base


def num_levels(cfg: Mapping) -> int:
    cur = cfg["curriculum"]
    return max(1, len(cur["levels"])) if cur.get("enabled", True) else 1
