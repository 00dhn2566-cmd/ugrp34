"""Batched convex QP layer on the GPU: OSQP's ADMM (Stellato et al. 2020) for many small problems at once.

    minimise 0.5 x'Px + q'x   subject to   l <= Ax <= u        (per batch element; l = -inf / u = +inf allowed)

P (B,n,n) positive semidefinite, A (B,m,n). One Cholesky factor of K = P + sigma I + rho A'A per solve, then a
fixed number of ADMM iterations (each a triangular solve, a clamp and a dual update) — the same algorithm as the
``osqp`` package with a fixed step rho, without its adaptive rho or polishing. Plain torch ops, so the solution
is differentiable through the unrolled iterations (a convex optimisation layer). Warm start with (x, z, y) from
the previous decision. ``solve`` returns the iterate and the primal / dual residuals (inf-norms) for checks.
"""
from __future__ import annotations

import torch


def solve_qp(P, q, A, l, u, iters: int = 60, rho: float = 1.0, sigma: float = 1e-6, alpha: float = 1.6,
             warm=None):
    """-> (x (B,n), z (B,m), y (B,m), primal residual (B,), dual residual (B,))."""
    B, n = q.shape
    m = A.shape[1]
    At = A.transpose(-1, -2)
    K = P + sigma * torch.eye(n, device=q.device, dtype=q.dtype) + rho * At @ A
    Lc = torch.linalg.cholesky(K)
    if warm is None:
        x = torch.zeros(B, n, device=q.device, dtype=q.dtype)
        z = torch.zeros(B, m, device=q.device, dtype=q.dtype)
        y = torch.zeros(B, m, device=q.device, dtype=q.dtype)
    else:
        x, z, y = warm
    for _ in range(iters):
        rhs = sigma * x - q + (At @ (rho * z - y)[..., None])[..., 0]
        xt = torch.cholesky_solve(rhs[..., None], Lc)[..., 0]
        zt = (A @ xt[..., None])[..., 0]
        x = alpha * xt + (1 - alpha) * x
        zr = alpha * zt + (1 - alpha) * z
        z_new = torch.minimum(torch.maximum(zr + y / rho, l), u)
        y = y + rho * (zr - z_new)
        z = z_new
    Ax = (A @ x[..., None])[..., 0]
    r_prim = (Ax - z).abs().amax(-1)
    r_dual = ((P @ x[..., None])[..., 0] + q + (At @ y[..., None])[..., 0]).abs().amax(-1)
    return x, z, y, r_prim, r_dual
