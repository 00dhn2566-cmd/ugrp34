"""Train a PPO policy on WindowTraversalAviary (real PyBullet quadrotor physics).

    python3 rl/train_pybullet.py --timesteps 600000 --n-envs 8 --n-windows 3

Logs a pass-rate curve: every --eval-freq steps it rolls out deterministic episodes
on fresh seeds and reports success rate + mean windows passed. Saves the model +
an eval CSV under runs/. Physics is CPU-bound (PyBullet), parallelised across
--n-envs cores; the small MLP policy trains on the GPU if available.
"""
from __future__ import annotations

import argparse
import csv
import os
import sys

import numpy as np

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from rl.pybullet_window_env import WindowTraversalAviary  # noqa: E402
from stable_baselines3 import PPO  # noqa: E402
from stable_baselines3.common.vec_env import SubprocVecEnv, VecMonitor  # noqa: E402
from stable_baselines3.common.callbacks import BaseCallback  # noqa: E402


def make_env(rank: int, n_windows: int, opening: float, step: float):
    def _f():
        return WindowTraversalAviary(n_windows=n_windows, seed=1000 + rank,
                                     opening=opening, step=step)
    return _f


def evaluate(model, n_windows: int, n_eps: int, opening: float, step: float, seed0: int = 90000):
    """Deterministic rollouts on fresh seeds -> (success_rate, mean_windows_passed)."""
    env = WindowTraversalAviary(n_windows=n_windows, seed=seed0, opening=opening, step=step)
    succ, passed = 0, []
    for e in range(n_eps):
        obs, _ = env.reset(seed=seed0 + e)
        done = False
        info = {"windows_passed": 0}
        while not done:
            act, _ = model.predict(obs, deterministic=True)
            obs, _, term, trunc, info = env.step(act)
            done = term or trunc
        succ += int(term)  # terminated == all windows cleared
        passed.append(info["windows_passed"])
    env.close()
    return succ / n_eps, float(np.mean(passed))


class PassRateCallback(BaseCallback):
    def __init__(self, n_windows, eval_freq, n_eps, csv_path, opening, step, verbose=0):
        super().__init__(verbose)
        self.n_windows, self.eval_freq, self.n_eps, self.csv_path = n_windows, eval_freq, n_eps, csv_path
        self.opening, self.step = opening, step
        self._next = eval_freq
        with open(csv_path, "w", newline="") as f:
            csv.writer(f).writerow(["timesteps", "success_rate", "mean_windows_passed"])

    def _on_step(self) -> bool:
        if self.num_timesteps >= self._next:
            self._next += self.eval_freq
            sr, mp = evaluate(self.model, self.n_windows, self.n_eps, self.opening, self.step)
            with open(self.csv_path, "a", newline="") as f:
                csv.writer(f).writerow([self.num_timesteps, f"{sr:.3f}", f"{mp:.2f}"])
            print(f"[eval] t={self.num_timesteps:>8}  success={sr:5.1%}  mean_passed={mp:.2f}/{self.n_windows}",
                  flush=True)
        return True


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--timesteps", type=int, default=600_000)
    ap.add_argument("--n-envs", type=int, default=8)
    ap.add_argument("--n-windows", type=int, default=3)
    ap.add_argument("--eval-freq", type=int, default=50_000)
    ap.add_argument("--eval-eps", type=int, default=20)
    ap.add_argument("--opening", type=float, default=0.35)
    ap.add_argument("--step", type=float, default=0.6)
    ap.add_argument("--init-from", default=None, help="warm-start policy .zip (curriculum).")
    ap.add_argument("--out", default=os.path.join(_ROOT, "runs", "ppo_window_pyb"))
    args = ap.parse_args(argv)
    os.makedirs(args.out, exist_ok=True)

    venv = VecMonitor(SubprocVecEnv([make_env(i, args.n_windows, args.opening, args.step)
                                     for i in range(args.n_envs)]))
    if args.init_from:
        print(f"[train] warm-start from {args.init_from}", flush=True)
        model = PPO.load(args.init_from, env=venv, device="cuda")
    else:
        model = PPO("MlpPolicy", venv, device="cuda", n_steps=2048, batch_size=512,
                    gae_lambda=0.95, gamma=0.99, ent_coef=0.005, learning_rate=3e-4,
                    policy_kwargs=dict(net_arch=[128, 128]), verbose=1)
    cb = PassRateCallback(args.n_windows, args.eval_freq, args.eval_eps,
                          os.path.join(args.out, "pass_rate.csv"), args.opening, args.step)
    print(f"[train] {args.timesteps} steps, {args.n_envs} envs, {args.n_windows} windows -> {args.out}", flush=True)
    model.learn(total_timesteps=args.timesteps, callback=cb, progress_bar=False)
    model.save(os.path.join(args.out, "policy.zip"))
    sr, mp = evaluate(model, args.n_windows, 50, args.opening, args.step)
    print(f"[final] success={sr:.1%}  mean_passed={mp:.2f}/{args.n_windows}", flush=True)
    print("TRAIN_PYB_DONE", flush=True)


if __name__ == "__main__":
    main()
