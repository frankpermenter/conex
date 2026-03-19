#!/usr/bin/env python3
"""Profile Conex least-squares on SuiteSparse matrices.

Compares four solver paths:
  conex (row-partition)  — sparse_ls_profile_csr
  conex (implicit)       — sparse_ls_profile_csr_implicit
  conex (NE)             — sparse_ls_ne (SparseLinearConstraint)
  scipy                  — spsolve / splu
"""
import argparse
import time
from pathlib import Path

import numpy as np
import scipy.io
import scipy.sparse as sp
import scipy.sparse.linalg as spla
from _conex import sparse_ls_profile_csr, sparse_ls_profile_csr_implicit
from conex import sparse_ls_ne


def load_mtx(path: Path) -> sp.csr_matrix:
    A = scipy.io.mmread(str(path))
    if not sp.issparse(A):
        A = sp.coo_matrix(A)
    A = A.tocsr()
    if np.iscomplexobj(A.data):
        raise ValueError("Complex matrices are not supported by this script.")
    A.sum_duplicates()
    return A.astype(np.float64, copy=False)


def find_smallest_mtx(root: Path) -> Path:
    candidates = list(root.rglob("*.mtx"))
    if not candidates:
        raise FileNotFoundError(f"No .mtx files found under: {root}")
    best = None
    best_key = None
    for p in candidates:
        A = load_mtx(p)
        key = (A.shape[0] * A.shape[1], A.shape[0] + A.shape[1],
               A.shape[0], A.shape[1], str(p))
        if best_key is None or key < best_key:
            best_key = key
            best = p
    return best


def time_median(fn, reps: int):
    times = []
    out = None
    for _ in range(reps):
        t0 = time.perf_counter()
        out = fn()
        times.append((time.perf_counter() - t0) * 1000.0)
    times.sort()
    return times[len(times) // 2], out


def fmt_ms(v):
    """Format milliseconds: use us for < 0.1 ms."""
    if v < 0.1:
        return f"{v * 1000:6.1f} us"
    return f"{v:6.3f} ms"


def rel_error(x, x_ref):
    return np.linalg.norm(x - x_ref) / max(1.0, np.linalg.norm(x_ref))


def rel_residual(M, x, rhs):
    return np.linalg.norm(M @ x - rhs) / max(1.0, np.linalg.norm(rhs))


def profile_scipy_reference(ata_csc, rhs, reps: int):
    spsolve_ms, x_spsolve = time_median(
        lambda: spla.spsolve(ata_csc, rhs), reps=max(1, reps))

    factor_times, solve_times = [], []
    x_lu = None
    for _ in range(max(1, reps)):
        t0 = time.perf_counter()
        lu = spla.splu(ata_csc)
        t1 = time.perf_counter()
        x_lu = lu.solve(rhs)
        t2 = time.perf_counter()
        factor_times.append((t1 - t0) * 1000.0)
        solve_times.append((t2 - t1) * 1000.0)
    factor_times.sort()
    solve_times.sort()
    return {
        "x_spsolve": np.asarray(x_spsolve, dtype=np.float64).reshape(-1),
        "spsolve_total_ms": spsolve_ms,
        "splu_factor_ms": factor_times[len(factor_times) // 2],
        "splu_solve_ms": solve_times[len(solve_times) // 2],
        "x_splu": np.asarray(x_lu, dtype=np.float64).reshape(-1),
    }


def profile_maximal_chordal(Acsr, b, num_threads):
    prof = sparse_ls_profile_csr_implicit(
        Acsr.indptr.astype(np.int64, copy=False),
        Acsr.indices.astype(np.int64, copy=False),
        Acsr.data.astype(np.float64, copy=False),
        int(Acsr.shape[0]),
        int(Acsr.shape[1]),
        np.asarray(b, dtype=np.float64),
        int(num_threads),
    )
    return {
        "x": np.asarray(prof["x"], dtype=np.float64).reshape(-1),
        "num_cliques": int(prof["num_cliques"]),
        "support_ms": float(prof["support_ms"]),
        "blocks_ms": float(prof["blocks_ms"]),
        "finalize_ms": float(prof["finalize_ms"]),
        "factor_ms": float(prof["factor_ms"]),
        "solve_ms": float(prof["solve_ms"]),
        "total_ms": float(prof["total_ms"]),
    }


def main():
    parser = argparse.ArgumentParser(
        description="Profile Conex least-squares on the smallest local SuiteSparse matrix."
    )
    parser.add_argument(
        "--suitesparse-dir", type=str, default="data/suitesparse",
        help="Root directory containing extracted SuiteSparse .mtx files.",
    )
    parser.add_argument(
        "--matrix", type=str, default=None,
        help="Explicit matrix path (.mtx). If omitted, the smallest matrix is selected.",
    )
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--num-threads", type=int, default=8)
    parser.add_argument("--reps", type=int, default=3)
    args = parser.parse_args()

    if args.matrix is not None:
        matrix_path = Path(args.matrix)
    else:
        matrix_path = find_smallest_mtx(Path(args.suitesparse_dir))

    A = load_mtx(matrix_path)
    m, n = A.shape
    nnz = A.nnz
    rng = np.random.default_rng(args.seed)
    x_true = rng.normal(0.0, 1.0, size=n)
    b = A @ x_true

    reg = 1e-8
    ata = (A.T @ A) + reg * sp.eye(n, format="csr")
    rhs = A.T @ b
    ata_csc = ata.tocsc()

    # ── Header ───────────────────────────────────────────────────────────
    print(f"{'=' * 74}")
    print(f"  Matrix: {matrix_path.name}")
    print(f"  Shape:  {m} x {n}    nnz(A) = {nnz}    nnz(A^TA) = {ata.nnz}")
    print(f"  Threads: {args.num_threads}    Reps: {args.reps}    Seed: {args.seed}")
    print(f"{'=' * 74}")

    # ── Conex row-partition path ─────────────────────────────────────────
    prof = sparse_ls_profile_csr(
        A.indptr.astype(np.int64, copy=False),
        A.indices.astype(np.int64, copy=False),
        A.data.astype(np.float64, copy=False),
        int(m), int(n),
        np.asarray(b, dtype=np.float64),
        int(args.num_threads),
    )
    x_conex = np.asarray(prof["x"], dtype=np.float64).reshape(-1)

    print(f"\n  Conex row-partition  ({int(prof['num_cliques'])} cliques"
          f"{', single dense' if bool(prof['single_dense_clique']) else ''})")
    print(f"  {'Phase':<20} {'Time':>10}")
    print(f"  {'-' * 20} {'-' * 10}")
    print(f"  {'support':<20} {fmt_ms(float(prof['support_ms'])):>10}")
    print(f"  {'blocks':<20} {fmt_ms(float(prof['blocks_ms'])):>10}")
    print(f"  {'finalize':<20} {fmt_ms(float(prof['finalize_ms'])):>10}")
    print(f"  {'factor':<20} {fmt_ms(float(prof['factor_ms'])):>10}")
    print(f"  {'solve':<20} {fmt_ms(float(prof['solve_ms'])):>10}")
    print(f"  {'TOTAL':<20} {fmt_ms(float(prof['total_ms'])):>10}")

    # ── Conex implicit maximal-clique path ───────────────────────────────
    maximal_prof_ms, maximal_prof = time_median(
        lambda: profile_maximal_chordal(A, b, int(args.num_threads)),
        reps=max(1, int(args.reps)),
    )
    x_maximal = maximal_prof["x"]

    print(f"\n  Conex implicit  ({int(maximal_prof['num_cliques'])} cliques)")
    print(f"  {'Phase':<20} {'Time':>10}")
    print(f"  {'-' * 20} {'-' * 10}")
    print(f"  {'support':<20} {fmt_ms(maximal_prof['support_ms']):>10}")
    print(f"  {'blocks':<20} {fmt_ms(maximal_prof['blocks_ms']):>10}")
    print(f"  {'finalize':<20} {fmt_ms(maximal_prof['finalize_ms']):>10}")
    print(f"  {'factor':<20} {fmt_ms(maximal_prof['factor_ms']):>10}")
    print(f"  {'solve':<20} {fmt_ms(maximal_prof['solve_ms']):>10}")
    print(f"  {'TOTAL':<20} {fmt_ms(maximal_prof['total_ms']):>10}")
    print(f"  {'total (median)':<20} {fmt_ms(maximal_prof_ms):>10}")

    # ── Conex NE path ────────────────────────────────────────────────────
    ne_prof_ms, ne_prof = time_median(
        lambda: sparse_ls_ne(A, rhs),
        reps=max(1, int(args.reps)),
    )
    x_ne = np.asarray(ne_prof["x"], dtype=np.float64).reshape(-1)
    ne_total_ms = (ne_prof["construction_us"] + ne_prof["assemble_and_factor_us"]
                   + ne_prof["solve_us"]) / 1000.0

    print(f"\n  Conex NE path")
    print(f"  {'Phase':<20} {'Time':>10}")
    print(f"  {'-' * 20} {'-' * 10}")
    print(f"  {'grouping':<20} {fmt_ms(ne_prof['grouping_us'] / 1000.0):>10}")
    print(f"  {'add constraints':<20} {fmt_ms(ne_prof['add_constraints_us'] / 1000.0):>10}")
    print(f"  {'init workspace':<20} {fmt_ms(ne_prof['init_workspace_us'] / 1000.0):>10}")
    print(f"  {'clique extraction':<20} {fmt_ms(ne_prof['clique_extraction_us'] / 1000.0):>10}")
    print(f"  {'finalize':<20} {fmt_ms(ne_prof['finalize_us'] / 1000.0):>10}")
    print(f"  {'construction':<20} {fmt_ms(ne_prof['construction_us'] / 1000.0):>10}")
    print(f"  {'factor':<20} {fmt_ms(ne_prof['assemble_and_factor_us'] / 1000.0):>10}")
    print(f"  {'solve':<20} {fmt_ms(ne_prof['solve_us'] / 1000.0):>10}")
    print(f"  {'TOTAL':<20} {fmt_ms(ne_total_ms):>10}")
    print(f"  {'total (median)':<20} {fmt_ms(ne_prof_ms):>10}")

    # ── Scipy reference ──────────────────────────────────────────────────
    scipy_prof = profile_scipy_reference(ata_csc, rhs, reps=max(1, int(args.reps)))
    x_scipy = scipy_prof["x_spsolve"]

    print(f"\n  Scipy reference")
    print(f"  {'Phase':<20} {'Time':>10}")
    print(f"  {'-' * 20} {'-' * 10}")
    print(f"  {'spsolve (total)':<20} {fmt_ms(scipy_prof['spsolve_total_ms']):>10}")
    print(f"  {'splu factor':<20} {fmt_ms(scipy_prof['splu_factor_ms']):>10}")
    print(f"  {'splu solve':<20} {fmt_ms(scipy_prof['splu_solve_ms']):>10}")

    # ── Comparison table ─────────────────────────────────────────────────
    res_conex = rel_residual(ata, x_conex, rhs)
    res_maximal = rel_residual(ata, x_maximal, rhs)
    res_ne = rel_residual(ata, x_ne, rhs)
    res_scipy = rel_residual(ata, x_scipy, rhs)

    print(f"\n  {'Solver':<20} {'Total':>10}  {'|x - x_scipy|/|x|':>18}  {'|Ax-b|/|b|':>12}")
    print(f"  {'-' * 20} {'-' * 10}  {'-' * 18}  {'-' * 12}")
    print(f"  {'conex row-part':<20} {fmt_ms(float(prof['total_ms'])):>10}"
          f"  {rel_error(x_conex, x_scipy):18.2e}  {res_conex:12.2e}")
    print(f"  {'conex implicit':<20} {fmt_ms(maximal_prof['total_ms']):>10}"
          f"  {rel_error(x_maximal, x_scipy):18.2e}  {res_maximal:12.2e}")
    print(f"  {'conex NE':<20} {fmt_ms(ne_total_ms):>10}"
          f"  {rel_error(x_ne, x_scipy):18.2e}  {res_ne:12.2e}")
    print(f"  {'scipy spsolve':<20} {fmt_ms(scipy_prof['spsolve_total_ms']):>10}"
          f"  {'—':>18}  {res_scipy:12.2e}")
    print()


if __name__ == "__main__":
    main()
