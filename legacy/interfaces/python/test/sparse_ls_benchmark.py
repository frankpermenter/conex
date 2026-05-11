#!/usr/bin/env python3
"""Benchmark sparse least-squares solvers.

Compares four approaches on star-graph test matrices:
  tree  — conex implicit-clique tree solver (sparse_ls)
  ne    — conex normal-equations via SparseLinearConstraint (sparse_ls_ne)
  lsqr  — scipy iterative LSQR
  dense — numpy dense solve of A^T A x = A^T b
"""
import time

import numpy as np
import scipy.sparse as sp
import scipy.sparse.linalg as spla

import conex


def build_star_ls_matrix(num_leaves: int, repeats: int) -> sp.csr_matrix:
    """Star graph: variable 0 is the center, 1..num_leaves are leaves."""
    n = num_leaves + 1
    rows, cols, vals = [], [], []
    r = 0

    rows.append(r); cols.append(0); vals.append(2.0); r += 1

    for rep in range(repeats):
        scale = 0.7 + 0.2 * (rep % 3)
        for leaf in range(1, n):
            rows.extend([r, r])
            cols.extend([0, leaf])
            vals.extend([1.0, scale])
            r += 1

    for leaf in range(1, n):
        rows.append(r); cols.append(leaf)
        vals.append(1.0 + 0.01 * (leaf % 11))
        r += 1

    return sp.csr_matrix((vals, (rows, cols)), shape=(r, n))


def rel_error(x, x_ref):
    return np.linalg.norm(x - x_ref) / max(1.0, np.linalg.norm(x_ref))


def residual_norm(A, x, b):
    return np.linalg.norm(A @ x - b)


def time_call(fn, repeats=5):
    best_ms = float("inf")
    out = None
    for _ in range(repeats):
        t0 = time.perf_counter()
        out = fn()
        best_ms = min(best_ms, (time.perf_counter() - t0) * 1000.0)
    return best_ms, out


def fmt_ms(v):
    """Format milliseconds: use us for < 0.1 ms."""
    if v < 0.1:
        return f"{v * 1000:6.1f} us"
    return f"{v:6.3f} ms"


def main():
    instances = [
        (64, 6),
        (128, 8),
        (256, 10),
        (384, 12),
        (512, 14),
    ]

    for leaves, repeats in instances:
        A = build_star_ls_matrix(leaves, repeats)
        n = A.shape[1]
        m = A.shape[0]
        x_true = np.linspace(-0.8, 1.2, n)
        b = A @ x_true
        ata = (A.T @ A).tocsr()
        rhs = A.T @ b

        # -- solvers ----------------------------------------------------------
        tree_ms, x_tree = time_call(
            lambda: np.asarray(conex.sparse_ls(A, b, num_threads=1), dtype=np.float64))

        ne_ms, ne_result = time_call(lambda: conex.sparse_ls_ne(A, rhs))
        x_ne = np.asarray(ne_result["x"]).reshape(-1)
        ne_build = ne_result["construction_us"] / 1000.0
        ne_factor = ne_result["assemble_and_factor_us"] / 1000.0
        ne_solve = ne_result["solve_us"] / 1000.0
        ne_grouping = ne_result["grouping_us"] / 1000.0
        ne_add = ne_result["add_constraints_us"] / 1000.0
        ne_init = ne_result["init_workspace_us"] / 1000.0
        ne_clique = ne_result["clique_extraction_us"] / 1000.0
        ne_finalize = ne_result["finalize_us"] / 1000.0

        lsqr_ms, x_lsqr = time_call(
            lambda: spla.lsqr(A, b, atol=1e-14, btol=1e-13, iter_lim=40000)[0])

        dense_ms, x_dense = time_call(
            lambda: np.linalg.solve(ata.toarray(), rhs))

        # -- print -------------------------------------------------------------
        print(f"{'=' * 70}")
        print(f"  n = {n},  m = {m},  nnz(A) = {A.nnz},  nnz(A^TA) = {ata.nnz}")
        print(f"{'=' * 70}")

        print(f"\n  {'Solver':<10} {'Total':>10} {'Detail':>40}  {'|x - x*|/|x*|':>14}")
        print(f"  {'-' * 10} {'-' * 10} {'-' * 40}  {'-' * 14}")

        err_tree = rel_error(x_tree, x_true)
        print(f"  {'tree':<10} {fmt_ms(tree_ms):>10} {'':>40}  {err_tree:14.2e}")

        err_ne = rel_error(x_ne, x_true)
        detail = f"build {fmt_ms(ne_build)}  fact {fmt_ms(ne_factor)}  solve {fmt_ms(ne_solve)}"
        print(f"  {'ne':<10} {fmt_ms(ne_ms):>10} {detail:>40}  {err_ne:14.2e}")

        print(f"\n  NE build breakdown ({fmt_ms(ne_build).strip()} total):")
        print(f"    grouping (SparseLinearConstraint): {fmt_ms(ne_grouping)}")
        print(f"    add_constraints (to CM):           {fmt_ms(ne_add)}")
        print(f"    init_workspace + SetIdentity:      {fmt_ms(ne_init)}")
        print(f"    clique extraction + tree build:    {fmt_ms(ne_clique)}")
        print(f"    finalize (adapters + Finalize):    {fmt_ms(ne_finalize)}")

        err_lsqr = rel_error(x_lsqr, x_true)
        print(f"  {'lsqr':<10} {fmt_ms(lsqr_ms):>10} {'':>40}  {err_lsqr:14.2e}")

        err_dense = rel_error(x_dense, x_true)
        print(f"  {'dense':<10} {fmt_ms(dense_ms):>10} {'':>40}  {err_dense:14.2e}")

        print(f"\n  Speedup vs ne:  tree {ne_ms/tree_ms:.2f}x   "
              f"lsqr {ne_ms/lsqr_ms:.2f}x   dense {ne_ms/dense_ms:.2f}x")

        print(f"\n  Residual ||Ax - b||:")
        print(f"    tree  {residual_norm(A, x_tree, b):.2e}    "
              f"ne  {residual_norm(A, x_ne, b):.2e}    "
              f"lsqr  {residual_norm(A, x_lsqr, b):.2e}    "
              f"dense  {residual_norm(A, x_dense, b):.2e}")
        print()


if __name__ == "__main__":
    main()
