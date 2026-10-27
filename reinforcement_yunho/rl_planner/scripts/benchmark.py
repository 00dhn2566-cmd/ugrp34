"""General-robustness benchmark: many conditions x many planners on the same seeded scenes (RESULTS.md §10).

    # run (resumable: finished condition/planner pairs are skipped; --shard k/n splits the conditions)
    python rl_planner/scripts/benchmark.py --out /data/runs/rl_planner/bench_general \\
        --fsm-config /data/runs/rl_planner/fsm_tuning/best.yaml --planner fsm --planner fsm+safe \\
        --policy main=<run>/export --planner main:rl --planner main:rl+safe
    # table (success % per condition and tier; paired McNemar of --pair a,b)
    python rl_planner/scripts/benchmark.py --table /data/runs/rl_planner/bench_general --pair gen:rl+safe,fsm+safe

Conditions come from ``configs/general/suite.yaml`` (level + partial yamls + tier). Every planner of a
condition runs on an env reseeded the same way, and the scene fingerprint must match across planners and
policies (same scenes, camera noise and disturbances), so any two columns are a same-scene comparison.
Per condition and planner: <out>/<condition>/<planner>.json (summary + meta) and .csv (episodes).
Runs on the CPU by default (4 threads; ~5 s per planner and condition — the per-step work is small, a
busy GPU is slower). CPU and CUDA draw different random scenes, so keep one device per result root.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

CONFIGS = os.path.join(os.path.dirname(HERE), "configs")
SUITE = os.path.join(CONFIGS, "general", "suite.yaml")
TIERS = (("nominal", "명목"), ("long", "창문 50 개"), ("in_dr", "학습 DR 범위 안"), ("beyond", "학습 DR 범위 밖"),
         ("held_out", "학습에 없던 교란"), ("compound", "복합"), ("safety", "안전 (성공 불가)"))


def load_suite(path: str = SUITE) -> list:
    import yaml
    with open(path, encoding="utf-8") as f:
        return yaml.safe_load(f)["conditions"]


def _fname(planner: str) -> str:
    return planner.replace(":", "__").replace("+", "_plus_")


def run(a) -> int:
    import torch

    from rl_planner.core.config import config_for_policy, load_config
    from rl_planner.envs.torch_env import TorchWindowEnv
    from rl_planner.eval_harness import per_env_rows, run_planner, save_json, scene_fingerprint, summarize, write_csv
    from rl_planner.planners.fsm import FSMPlanner
    from rl_planner.planners.policy import PolicyPlanner
    from rl_planner.planners.safety import SafePlanner

    policies = dict(p.split("=", 1) for p in a.policy)
    planners = []
    for spec in a.planner:                        # "fsm", "fsm+safe", "<policy>:rl[+safe]", "<policy>:scan[+safe]"
        pol, _, kind = spec.rpartition(":")
        if pol and pol not in policies:
            raise SystemExit(f"--planner {spec}: unknown policy {pol!r} (give --policy {pol}=<export dir>)")
        planners.append((spec, pol or None, kind))
    conds = load_suite(a.suite)
    if a.only:
        keep = set(a.only.split(","))
        conds = [c for c in conds if c["name"] in keep]
    k, n_sh = (int(v) for v in a.shard.split("/"))
    conds = conds[k::n_sh]
    device = a.device                     # CPU by default: 512 small envs run faster there than on a busy GPU
    if device == "cpu":
        torch.set_num_threads(a.threads)  # fixed, so float reductions (and outcomes) are reproducible
    trained = {}
    for name, d in policies.items():
        with open(os.path.join(d, "policy_meta.json"), encoding="utf-8") as f:
            trained[name] = json.load(f)["config"]

    for c in conds:
        extra = [os.path.join(CONFIGS, p) for p in c.get("configs", [])]
        cdir = os.path.join(a.out, c["name"])
        todo = [p for p in planners if a.force or not os.path.exists(os.path.join(cdir, _fname(p[0]) + ".json"))]
        if not todo:
            continue
        fp_ref = None
        if os.path.isdir(cdir):                   # earlier planners of this condition fix the fingerprint
            for fn in os.listdir(cdir):
                if fn.endswith(".json"):
                    with open(os.path.join(cdir, fn), encoding="utf-8") as f:
                        fp_ref = json.load(f)["meta"]["scene_fingerprint"]
                    break
        groups = {}
        for p in todo:
            groups.setdefault(p[1], []).append(p)
        for pol, items in groups.items():
            fsm_extra = [a.fsm_config] if a.fsm_config else []
            if pol is None:
                cfg = load_config(fsm_extra + extra, a.set)
            else:
                cfg = config_for_policy(trained[pol], fsm_extra + extra, a.set)
            cfg["seed"] = a.seed
            cfg["sim"]["num_envs"] = a.num_scenes
            env = TorchWindowEnv(cfg, num_envs=a.num_scenes, device=device, seed=a.seed)
            for spec, _, kind in items:
                env.core.fixed_level = int(c["level"])
                env.core.reseed(a.seed)
                env._reset_all()
                fp = scene_fingerprint(env.core.scene)
                fp_ref = fp_ref or fp
                if fp != fp_ref:
                    raise SystemExit(f"[bench] {c['name']}/{spec}: different scenes ({fp} != {fp_ref})")
                base, _, guard = kind.partition("+")
                if base == "fsm":
                    planner = FSMPlanner(cfg, a.num_scenes, env.device)
                elif base == "rl":
                    planner = PolicyPlanner(policies[pol], device=env.device, cfg=cfg)
                elif base == "scan":                  # scripted look-around around a heading-fixed policy (§12)
                    from rl_planner.planners.scan import ScanSupervisor
                    planner = ScanSupervisor(PolicyPlanner(policies[pol], device=env.device, cfg=cfg), cfg,
                                             a.num_scenes, env.device, env.core.step_dt)
                else:
                    raise SystemExit(f"unknown planner kind {kind!r}")
                if guard == "safe":
                    planner = SafePlanner(planner, cfg, a.num_scenes, env.device, env.core.step_dt)
                rec = run_planner(env, planner, record_k=0)
                summ = summarize(rec)
                meta = {"condition": c["name"], "level": int(c["level"]), "configs": c.get("configs", []), "set": a.set,
                        "tier": c.get("tier"), "planner": spec, "policy": policies.get(pol), "seed": a.seed,
                        "num_scenes": a.num_scenes, "scene_fingerprint": fp, "device": device,
                        "threads": a.threads if device == "cpu" else None}
                os.makedirs(cdir, exist_ok=True)
                write_csv(os.path.join(cdir, _fname(spec) + ".csv"), per_env_rows(spec, rec))
                save_json(os.path.join(cdir, _fname(spec) + ".json"), {"meta": meta, "summary": summ})
                print(f"[bench] {c['name']:22s} {spec:16s} success={summ['success_rate']:.3f} "
                      f"crash={summ['crash_rate']:.3f} time={summ['mean_time_success_s']:.2f}", flush=True)
            env.close()
    return 0


# ----------------------------------------------------------------------------- tables
def _load(root: str, cond: str, planner: str):
    p = os.path.join(root, cond, _fname(planner) + ".json")
    if not os.path.exists(p):
        return None, None
    with open(p, encoding="utf-8") as f:
        s = json.load(f)["summary"]
    with open(os.path.join(root, cond, _fname(planner) + ".csv"), newline="") as f:
        ok = {int(r["env"]): r["success"] == "True" for r in csv.DictReader(f)}
    return s, ok


def mcnemar_p(b: int, c: int) -> float:
    n = b + c
    if n == 0:
        return 1.0
    tail = sum(math.comb(n, i) for i in range(min(b, c) + 1)) / 2 ** n
    return min(1.0, 2 * tail)


def table(root: str, cols: list, labels: dict, pairs: list, suite: str = SUITE) -> str:
    conds = load_suite(suite)
    out = ["| 조건 | " + " | ".join(labels.get(c, c) for c in cols) + " |"
           + "".join(f" {labels.get(x, x)} vs {labels.get(y, y)} |" for x, y in pairs),
           "|---|" + "---|" * (len(cols) + len(pairs))]
    agg = {}
    for tier, tier_name in TIERS:
        rows = [c for c in conds if c.get("tier") == tier]
        if not rows:
            continue
        out.append(f"| **{tier_name}** |" + " |" * (len(cols) + len(pairs)))
        for c in rows:
            cells, oks = [], {}
            for col in cols:
                s, ok = _load(root, c["name"], col)
                oks[col] = ok
                if s is None:
                    cells.append("–")
                    continue
                v = 100 * (s["crash_rate"] if tier == "safety" else s["success_rate"])
                cells.append(f"{v:.1f}" + (" (충돌)" if tier == "safety" else ""))
                if tier != "safety":
                    agg.setdefault((tier, col), []).append(s["success_rate"])
            pc = []
            for x, y in pairs:
                if oks.get(x) is None or oks.get(y) is None:
                    pc.append("–")
                    continue
                b = sum(1 for e in oks[x] if oks[x][e] and not oks[y].get(e, False))
                cc = sum(1 for e in oks[y] if oks[y][e] and not oks[x].get(e, False))
                p = mcnemar_p(b, cc)
                pc.append(f"{b} : {cc} ({'p<1e-4' if p < 1e-4 else f'p={p:.2g}'})")
            out.append(f"| {c['label']} | " + " | ".join(cells) + " |" + "".join(f" {x} |" for x in pc))
    out.append("")
    out.append("| 묶음 평균 / 최저 성공률 | " + " | ".join(labels.get(c, c) for c in cols) + " |")
    out.append("|---|" + "---|" * len(cols))
    for tier, tier_name in TIERS:
        if tier == "safety" or not any((tier, c) in agg for c in cols):
            continue
        cells = []
        for col in cols:
            v = agg.get((tier, col))
            cells.append("–" if not v else f"{100 * sum(v) / len(v):.1f} / {100 * min(v):.1f}")
        out.append(f"| {tier_name} | " + " | ".join(cells) + " |")
    return "\n".join(out)


KR_FONT = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
# failure-rate bins and the sequential blue ramp (steps 100 / 250 / 400 / 550 / 700) — light = few failures
FAIL_BINS = (0.0, 1.0, 3.0, 10.0, 30.0, 100.0)
FAIL_COLORS = ("#cde2fb", "#86b6ef", "#3987e5", "#1c5cab", "#0d366b")
FAIL_TEXT = ("#0b0b0b", "#0b0b0b", "#ffffff", "#ffffff", "#ffffff")


def figure(root: str, cols: list, labels: dict, path: str, suite: str = SUITE) -> None:
    """Heatmap: rows = conditions (grouped by tier, safety rows left out), columns = planners; cell colour =
    failure rate bin, cell text = success %."""
    import numpy as np

    sys.path.insert(0, HERE)
    from report import plt  # noqa: E402  (Agg backend)
    from matplotlib import font_manager
    from matplotlib.colors import BoundaryNorm, ListedColormap
    from matplotlib.patches import Patch
    if os.path.exists(KR_FONT):           # Korean labels (Noto Sans CJK carries Hangul in every regional face)
        font_manager.fontManager.addfont(KR_FONT)
        plt.rcParams["font.family"] = font_manager.FontProperties(fname=KR_FONT).get_name()

    conds = [c for c in load_suite(suite) if c.get("tier") != "safety"]
    order = [t for t, _ in TIERS]
    conds.sort(key=lambda c: order.index(c.get("tier")))
    val = np.full((len(conds), len(cols)), np.nan)
    for i, c in enumerate(conds):
        for j, col in enumerate(cols):
            s, _ = _load(root, c["name"], col)
            if s is not None:
                val[i, j] = 100 * s["success_rate"]
    fail = 100 - val
    cmap, norm = ListedColormap(FAIL_COLORS), BoundaryNorm(FAIL_BINS, len(FAIL_COLORS))
    cmap.set_bad("#fcfcfb")                  # not evaluated: surface colour
    fig, ax = plt.subplots(figsize=(1.05 * len(cols) + 3.6, 0.27 * len(conds) + 1.6))
    ax.imshow(np.ma.masked_invalid(fail), cmap=cmap, norm=norm, aspect="auto")
    for i in range(len(conds)):
        for j in range(len(cols)):
            if np.isnan(val[i, j]):
                ax.text(j, i, "–", ha="center", va="center", fontsize=7, color="#898781")
                continue
            k = int(np.clip(np.searchsorted(FAIL_BINS, fail[i, j], side="right") - 1, 0, len(FAIL_TEXT) - 1))
            ax.text(j, i, f"{val[i, j]:.1f}", ha="center", va="center", fontsize=7, color=FAIL_TEXT[k])
    ax.set_xticks(range(len(cols)), [labels.get(c, c) for c in cols], fontsize=7.5, color="#52514e")
    ax.xaxis.tick_top()
    ax.set_yticks(range(len(conds)), [c["label"] for c in conds], fontsize=7.5, color="#52514e")
    ax.tick_params(which="both", length=0)
    # white gaps between cells and between tiers
    ax.set_xticks(np.arange(-0.5, len(cols), 1), minor=True)
    ax.set_yticks(np.arange(-0.5, len(conds), 1), minor=True)
    ax.grid(which="minor", color="#fcfcfb", lw=1.5)
    for i in range(1, len(conds)):
        if conds[i]["tier"] != conds[i - 1]["tier"]:
            ax.axhline(i - 0.5, color="#fcfcfb", lw=5)
    tier_name = dict(TIERS)
    for t in order:
        idx = [i for i, c in enumerate(conds) if c["tier"] == t]
        if idx:
            ax.text(len(cols) - 0.4, (idx[0] + idx[-1]) / 2, tier_name[t], rotation=-90, ha="left", va="center",
                    fontsize=7.5, color="#52514e")
    for sp in ax.spines.values():
        sp.set_visible(False)
    fig.legend([Patch(color=c) for c in FAIL_COLORS],
               [f"{a:g}–{b:g}%" for a, b in zip(FAIL_BINS[:-1], FAIL_BINS[1:])], title="실패율 (숫자 = 성공률 %)",
               loc="lower center", ncol=len(FAIL_COLORS), frameon=False, fontsize=7.5, title_fontsize=8,
               labelcolor="#52514e", bbox_to_anchor=(0.5, 0.0))
    fig.tight_layout(rect=(0, 0.05, 1, 1))
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    fig.savefig(path, dpi=130, facecolor="#fcfcfb")
    plt.close(fig)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--out", help="result root (run mode)")
    ap.add_argument("--suite", default=SUITE)
    ap.add_argument("--policy", action="append", default=[], help="name=<export dir>")
    ap.add_argument("--planner", action="append", default=[], help="fsm | fsm+safe | <policy>:rl | <policy>:rl+safe")
    ap.add_argument("--fsm-config", default=None, help="tuned state-machine yaml (merged into every condition)")
    ap.add_argument("--only", default=None, help="comma-separated condition names")
    ap.add_argument("--set", action="append", default=[], help="dotted override applied to every condition (sweeps)")
    ap.add_argument("--shard", default="0/1", help="k/n: run every n-th condition starting at k")
    ap.add_argument("--num-scenes", type=int, default=512)
    ap.add_argument("--seed", type=int, default=12345)
    ap.add_argument("--device", default="cpu", help="cpu (default) or cuda:0 — scenes differ between devices "
                    "(different RNG), so every planner of one result root must use the same device")
    ap.add_argument("--threads", type=int, default=4, help="torch CPU threads per process")
    ap.add_argument("--force", action="store_true", help="re-run finished condition/planner pairs")
    ap.add_argument("--table", default=None, help="print the markdown table of a result root")
    ap.add_argument("--cols", default=None, help="comma-separated planners (table columns)")
    ap.add_argument("--labels", default=None, help="comma-separated planner=label")
    ap.add_argument("--pair", action="append", default=[], help="a,b: same-scene comparison column")
    ap.add_argument("--figure", default=None, help="with --table: also write a failure-rate heatmap PNG here")
    a = ap.parse_args(argv)
    if a.table:
        cols = a.cols.split(",") if a.cols else sorted({fn[:-5].replace("__", ":").replace("_plus_", "+")
                                                         for d in os.listdir(a.table) if os.path.isdir(os.path.join(a.table, d))
                                                         for fn in os.listdir(os.path.join(a.table, d)) if fn.endswith(".json")})
        labels = dict(x.split("=", 1) for x in a.labels.split(",")) if a.labels else {}
        print(table(a.table, cols, labels, [tuple(p.split(",")) for p in a.pair], a.suite))
        if a.figure:
            figure(a.table, cols, labels, a.figure, a.suite)
        return 0
    if not a.out or not a.planner:
        raise SystemExit("--out and --planner are required (or --table <root>)")
    return run(a)


if __name__ == "__main__":
    raise SystemExit(main())
