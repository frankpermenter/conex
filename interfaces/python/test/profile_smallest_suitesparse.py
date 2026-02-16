#!/usr/bin/env python3
import argparse
import time
from pathlib import Path

import numpy as np
import scipy.io
import scipy.sparse as sp
import scipy.sparse.linalg as spla
from _conex import sparse_ls_profile_csr, sparse_ls_profile_csr_implicit


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
        key = (A.shape[0] * A.shape[1], A.shape[0] + A.shape[1], A.shape[0], A.shape[1], str(p))
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


def profile_scipy_reference(ata_csc, rhs, reps: int):
    # Reference 1: direct one-shot solve.
    spsolve_ms, x_spsolve = time_median(lambda: spla.spsolve(ata_csc, rhs), reps=max(1, reps))

    # Reference 2: explicit factor + solve split via SuperLU (when available).
    # This is the closest external analogue to factor/solve profiling.
    factor_times = []
    solve_times = []
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
        "bags_ms": 0.0,
        "blocks_ms": float(prof["blocks_ms"]),
        "tree_ms": 0.0,
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
        "--suitesparse-dir",
        type=str,
        default="data/suitesparse",
        help="Root directory containing extracted SuiteSparse .mtx files.",
    )
    parser.add_argument(
        "--matrix",
        type=str,
        default=None,
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
    rng = np.random.default_rng(args.seed)
    x_true = rng.normal(0.0, 1.0, size=n)
    b = A @ x_true

    prof = sparse_ls_profile_csr(
        A.indptr.astype(np.int64, copy=False),
        A.indices.astype(np.int64, copy=False),
        A.data.astype(np.float64, copy=False),
        int(m),
        int(n),
        np.asarray(b, dtype=np.float64),
        int(args.num_threads),
    )
    x_conex = np.asarray(prof["x"], dtype=np.float64).reshape(-1)

    # Profile the implicit maximal-clique path fully in C++.
    reg = 1e-8
    maximal_prof_ms, maximal_prof = time_median(
        lambda: profile_maximal_chordal(
            A,
            b,
            int(args.num_threads),
        ),
        reps=max(1, int(args.reps)),
    )
    x_maximal = np.asarray(maximal_prof["x"], dtype=np.float64).reshape(-1)

    # Compare against scipy sparse normal-equation solve.
    ata = (A.T @ A) + reg * sp.eye(n, format="csr")
    rhs = A.T @ b
    scipy_prof = profile_scipy_reference(ata.tocsc(), rhs, reps=max(1, int(args.reps)))
    x_scipy = scipy_prof["x_spsolve"]

    rel_vs_scipy = np.linalg.norm(x_conex - x_scipy) / max(1.0, np.linalg.norm(x_scipy))
    rel_maximal_vs_scipy = np.linalg.norm(x_maximal - x_scipy) / max(
        1.0, np.linalg.norm(x_scipy)
    )
    res_conex = np.linalg.norm(ata @ x_conex - rhs) / max(1.0, np.linalg.norm(rhs))
    res_maximal = np.linalg.norm(ata @ x_maximal - rhs) / max(
        1.0, np.linalg.norm(rhs)
    )
    res_scipy = np.linalg.norm(ata @ x_scipy - rhs) / max(1.0, np.linalg.norm(rhs))

    print(f"matrix={matrix_path}")
    print(f"shape=({m}, {n}) nnz={A.nnz}")
    print(f"num_threads={args.num_threads} reps={args.reps}")
    print(
        "conex_profile_ms "
        f"support={float(prof['support_ms']):.3f} "
        f"blocks={float(prof['blocks_ms']):.3f} "
        f"finalize={float(prof['finalize_ms']):.3f} "
        f"factor={float(prof['factor_ms']):.3f} "
        f"solve={float(prof['solve_ms']):.3f} "
        f"total={float(prof['total_ms']):.3f}"
    )
    print(
        f"conex_tree_structure num_cliques={int(prof['num_cliques'])} "
        f"single_dense_clique={bool(prof['single_dense_clique'])}"
    )
    print(
        "conex_maximal_chordal_profile_ms "
        f"support={float(maximal_prof['support_ms']):.3f} "
        f"bags={float(maximal_prof['bags_ms']):.3f} "
        f"blocks={float(maximal_prof['blocks_ms']):.3f} "
        f"tree={float(maximal_prof['tree_ms']):.3f} "
        f"finalize={float(maximal_prof['finalize_ms']):.3f} "
        f"factor={float(maximal_prof['factor_ms']):.3f} "
        f"solve={float(maximal_prof['solve_ms']):.3f} "
        f"total={float(maximal_prof['total_ms']):.3f} "
        f"total_median={maximal_prof_ms:.3f}"
    )
    print(f"conex_maximal_chordal_tree_structure num_cliques={int(maximal_prof['num_cliques'])}")
    print(
        "reference_scipy_ms "
        f"spsolve_total={float(scipy_prof['spsolve_total_ms']):.3f} "
        f"splu_factor={float(scipy_prof['splu_factor_ms']):.3f} "
        f"splu_solve={float(scipy_prof['splu_solve_ms']):.3f}"
    )
    print(
        f"quality rel_vs_scipy={rel_vs_scipy:.3e} "
        f"residual_conex={res_conex:.3e} "
        f"rel_maximal_vs_scipy={rel_maximal_vs_scipy:.3e} "
        f"residual_maximal={res_maximal:.3e} "
        f"residual_scipy={res_scipy:.3e}"
    )


if __name__ == "__main__":
    main()
