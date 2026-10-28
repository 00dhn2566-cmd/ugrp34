"""Batched QP layer (core/qp.py) against the osqp package, and its gradient through the unrolled iterations."""
from __future__ import annotations

import os
import sys

import numpy as np
import pytest
import torch

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(os.path.dirname(HERE)))

from rl_planner.core.qp import solve_qp  # noqa: E402


def _random_qps(B, n, m, seed=0):
    rng = np.random.default_rng(seed)
    P, q = np.zeros((B, n, n)), rng.normal(size=(B, n))
    A, l, u = rng.normal(size=(B, m, n)), np.zeros((B, m)), np.zeros((B, m))
    for b in range(B):
        M = rng.normal(size=(n, n))
        P[b] = M @ M.T / n + 0.5 * np.eye(n)
        s = A[b] @ (rng.normal(size=n) * 0.3)              # a feasible point
        l[b], u[b] = s - rng.uniform(0.05, 1.0, m), s + rng.uniform(0.05, 1.0, m)
        l[b, : m // 4] = -np.inf
    return P, q, A, l, u


def test_matches_osqp_on_random_feasible_problems():
    osqp = pytest.importorskip("osqp")
    sp = pytest.importorskip("scipy.sparse")
    P, q, A, l, u = _random_qps(16, 8, 24)
    ref = []
    for b in range(len(q)):
        pr = osqp.OSQP()
        pr.setup(sp.csc_matrix(np.triu(P[b])), q[b], sp.csc_matrix(A[b]), l[b], u[b], verbose=False,
                 eps_abs=1e-10, eps_rel=1e-10, max_iter=100000)
        ref.append(pr.solve().x)
    t = lambda a: torch.tensor(a, dtype=torch.float64)  # noqa: E731
    x, _, _, rp, rd = solve_qp(t(P), t(q), t(A), t(l), t(u), iters=1500)
    assert np.abs(x.numpy() - np.array(ref)).max() < 1e-5
    assert float(rp.max()) < 1e-6 and float(rd.max()) < 1e-6


def test_projection_onto_a_box_and_a_halfspace():
    """Closest point to (2, 2) in [-1, 1]^2 with x + y <= 1: (0.5, 0.5); a point already inside stays put."""
    P = torch.eye(2, dtype=torch.float64).expand(2, 2, 2)
    q = -torch.tensor([[2.0, 2.0], [0.2, -0.3]], dtype=torch.float64)
    A = torch.tensor([[1.0, 0.0], [0.0, 1.0], [1.0, 1.0]], dtype=torch.float64).expand(2, 3, 2)
    l = torch.tensor([-1.0, -1.0, -np.inf], dtype=torch.float64).expand(2, 3)
    u = torch.tensor([1.0, 1.0, 1.0], dtype=torch.float64).expand(2, 3)
    x, *_ = solve_qp(P, q, A, l, u, iters=500)
    assert torch.allclose(x, torch.tensor([[0.5, 0.5], [0.2, -0.3]], dtype=torch.float64), atol=1e-6)


def test_gradient_flows_through_the_layer():
    """d x*/d q for an active constraint: projecting (t, 2) onto x + y <= 1 moves along (1, -1)/2."""
    P = torch.eye(2, dtype=torch.float64)[None]
    t = torch.tensor(0.3, dtype=torch.float64, requires_grad=True)
    q = -torch.stack((t, torch.tensor(2.0, dtype=torch.float64)))[None]
    A = torch.tensor([[[1.0, 1.0]]], dtype=torch.float64)
    x, *_ = solve_qp(P, q, A, torch.full((1, 1), -np.inf, dtype=torch.float64),
                     torch.ones(1, 1, dtype=torch.float64), iters=400)
    x[0, 0].backward()
    assert float(t.grad) == pytest.approx(0.5, abs=1e-4)
