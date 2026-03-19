#!/usr/bin/env python3
"""Download SuiteSparse least-squares benchmark matrices and profile Conex solvers.

Usage (from interfaces/python/):
    PYTHONPATH=. python3 test/benchmark_suitesparse_ls.py
    PYTHONPATH=. python3 test/benchmark_suitesparse_ls.py --data-dir /tmp/ss_data --reps 5
"""
import argparse
import io
import os
import tarfile
import time
from pathlib import Path

import numpy as np
import scipy.io
import scipy.sparse as sp
import scipy.sparse.linalg as spla
from _conex import sparse_ls_profile_csr, sparse_ls_profile_csr_implicit
from conex import sparse_ls_ne, sparse_ls_ne_maxclique

# Exact benchmark matrices: (group, name, nrows, ncols, nnz)
BENCHMARK_MATRICES = [
    ("HB", "abb313", 313, 176, 1557),
    ("HB", "ash219", 219, 85, 438),
    ("HB", "ash292", 292, 292, 2208),
    ("HB", "ash331", 331, 104, 662),
    ("HB", "ash608", 608, 188, 1216),
    ("HB", "ash85", 85, 85, 523),
    ("HB", "ash958", 958, 292, 1916),
    ("HB", "illc1033", 1033, 320, 4719),
    ("HB", "illc1850", 1850, 712, 8636),
    ("HB", "well1033", 1033, 320, 4732),
    ("HB", "well1850", 1850, 712, 8755),
    ("NYPA", "Maragal_1", 32, 14, 234),
    ("NYPA", "Maragal_2", 555, 350, 4357),
    ("NYPA", "Maragal_3", 1690, 860, 18391),
]

DOWNLOAD_URL = "https://suitesparse-collection-website.herokuapp.com/MM"


def download_matrix(group: str, name: str, data_dir: Path) -> Path:
    """Download and extract a SuiteSparse matrix, returning the .mtx path."""
    mtx_path = data_dir / name / f"{name}.mtx"
    if mtx_path.exists():
        return mtx_path

    url = f"{DOWNLOAD_URL}/{group}/{name}.tar.gz"
    print(f"  Downloading {group}/{name} ...", end=" ", flush=True)

    import urllib.request
    try:
        with urllib.request.urlopen(url) as resp:
            tar_bytes = resp.read()
    except Exception as e:
        print(f"FAILED ({e})")
        return None

    dest = data_dir / name
    dest.mkdir(parents=True, exist_ok=True)
    with tarfile.open(fileobj=io.BytesIO(tar_bytes), mode="r:gz") as tf:
        for member in tf.getmembers():
            # Strip the top-level directory from the archive.
            parts = member.name.split("/", 1)
            if len(parts) < 2 or not parts[1]:
                continue
            member.name = parts[1]
            tf.extract(member, dest)

    if mtx_path.exists():
        print("OK")
        return mtx_path

    print("FAILED (mtx not found after extraction)")
    return None


def load_mtx(path: Path) -> sp.csr_matrix:
    A = scipy.io.mmread(str(path))
    if not sp.issparse(A):
        A = sp.coo_matrix(A)
    A = A.tocsr()
    if np.iscomplexobj(A.data):
        raise ValueError("Complex matrices not supported")
    A.sum_duplicates()
    return A.astype(np.float64, copy=False)


def time_median(fn, reps: int):
    times = []
    out = None
    for _ in range(reps):
        t0 = time.perf_counter()
        out = fn()
        times.append((time.perf_counter() - t0) * 1000.0)
    times.sort()
    return times[len(times) // 2], out


def profile_conex(Acsr, b, num_threads):
    return sparse_ls_profile_csr(
        Acsr.indptr.astype(np.int64, copy=False),
        Acsr.indices.astype(np.int64, copy=False),
        Acsr.data.astype(np.float64, copy=False),
        int(Acsr.shape[0]),
        int(Acsr.shape[1]),
        np.asarray(b, dtype=np.float64),
        int(num_threads),
    )


def profile_conex_implicit(Acsr, b, num_threads):
    return sparse_ls_profile_csr_implicit(
        Acsr.indptr.astype(np.int64, copy=False),
        Acsr.indices.astype(np.int64, copy=False),
        Acsr.data.astype(np.float64, copy=False),
        int(Acsr.shape[0]),
        int(Acsr.shape[1]),
        np.asarray(b, dtype=np.float64),
        int(num_threads),
    )


def profile_scipy(ata_csc, rhs, reps: int):
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
    mid = len(factor_times) // 2
    return {
        "x": np.asarray(x_lu, dtype=np.float64).reshape(-1),
        "factor_ms": factor_times[mid],
        "solve_ms": solve_times[mid],
        "total_ms": factor_times[mid] + solve_times[mid],
    }


def main():
    parser = argparse.ArgumentParser(
        description="Download SuiteSparse LS benchmarks and profile Conex solvers."
    )
    parser.add_argument(
        "--data-dir", type=str, default=None,
        help="Directory for downloaded matrices. Default: <script_dir>/benchmark_data",
    )
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--num-threads", type=int, default=8)
    parser.add_argument("--reps", type=int, default=3)
    parser.add_argument("--reg", type=float, default=0.0,
                        help="Ridge regularization added to A^T A.")
    args = parser.parse_args()

    if args.data_dir is not None:
        data_dir = Path(args.data_dir)
    else:
        data_dir = Path(__file__).resolve().parent / "benchmark_data"
    data_dir.mkdir(parents=True, exist_ok=True)

    # Download matrices.
    print(f"Data directory: {data_dir}")
    matrices = []
    for group, name, nrows, ncols, nnz in BENCHMARK_MATRICES:
        mtx_path = download_matrix(group, name, data_dir)
        if mtx_path is not None:
            matrices.append((name, nrows, ncols, nnz, mtx_path))

    if not matrices:
        print("ERROR: No matrices available.")
        return 1

    reg = args.reg
    rng = np.random.default_rng(args.seed)

    # Collect results.
    results = []
    for name, nrows, ncols, nnz, mtx_path in matrices:
        print(f"\nProfiling {name} ({nrows}x{ncols}, nnz={nnz}) ...", flush=True)
        A_raw = load_mtx(mtx_path)

        # Remove zero columns so every variable appears in at least one row.
        col_nnz = np.diff(A_raw.tocsc().indptr)
        live_cols = np.flatnonzero(col_nnz)
        if len(live_cols) < A_raw.shape[1]:
            A = A_raw[:, live_cols]
            A = sp.csr_matrix(A)
            print(f"  Removed {A_raw.shape[1] - len(live_cols)} zero columns "
                  f"({A_raw.shape[1]} -> {A.shape[1]})")
        else:
            A = A_raw

        m, n = A.shape
        x_true = rng.normal(0.0, 1.0, size=n)
        b = A @ x_true

        ata = (A.T @ A) + reg * sp.eye(n, format="csr")
        rhs = A.T @ b

        # Conex standard path.
        conex_prof = None
        try:
            _, conex_prof = time_median(
                lambda: profile_conex(A, b, args.num_threads), reps=args.reps
            )
        except Exception as e:
            print(f"  Conex row-partition failed: {e}")

        # Conex implicit (maximal chordal) path.
        implicit_prof = None
        try:
            _, implicit_prof = time_median(
                lambda: profile_conex_implicit(A, b, args.num_threads), reps=args.reps
            )
        except Exception as e:
            print(f"  Conex implicit failed: {e}")

        # Conex NE path (containment grouping).
        ne_prof = None
        try:
            _, ne_prof = time_median(
                lambda: sparse_ls_ne(A, rhs), reps=args.reps
            )
        except Exception as e:
            print(f"  NE path failed: {e}")

        # Conex NE path (maximal-clique grouping).
        ne_mc_prof = None
        try:
            _, ne_mc_prof = time_median(
                lambda: sparse_ls_ne_maxclique(A, rhs), reps=args.reps
            )
        except Exception as e:
            print(f"  NE maxclique path failed: {e}")

        # SciPy reference.
        scipy_prof = None
        try:
            scipy_prof = profile_scipy(ata.tocsc(), rhs, reps=args.reps)
        except Exception as e:
            print(f"  SciPy failed: {e}")

        x_scipy = scipy_prof["x"] if scipy_prof else None

        if conex_prof is not None:
            x_conex = np.asarray(conex_prof["x"], dtype=np.float64).reshape(-1)
            conex_ms = float(conex_prof["total_ms"])
            conex_cliques = int(conex_prof["num_cliques"])
            res_conex = np.linalg.norm(ata @ x_conex - rhs) / max(1.0, np.linalg.norm(rhs))
        else:
            conex_ms = None
            conex_cliques = None
            res_conex = None

        if implicit_prof is not None:
            x_implicit = np.asarray(implicit_prof["x"], dtype=np.float64).reshape(-1)
            implicit_ms = float(implicit_prof["total_ms"])
            implicit_cliques = int(implicit_prof["num_cliques"])
            res_implicit = np.linalg.norm(ata @ x_implicit - rhs) / max(1.0, np.linalg.norm(rhs))
        else:
            implicit_ms = None
            implicit_cliques = None
            res_implicit = None

        if ne_prof is not None:
            x_ne = np.asarray(ne_prof["x"], dtype=np.float64).reshape(-1)
            ne_total_ms = (ne_prof["construction_us"] + ne_prof["assemble_and_factor_us"]
                           + ne_prof["solve_us"]) / 1000.0
            res_ne = np.linalg.norm(ata @ x_ne - rhs) / max(1.0, np.linalg.norm(rhs))
        else:
            ne_total_ms = None
            res_ne = None

        if ne_mc_prof is not None:
            x_ne_mc = np.asarray(ne_mc_prof["x"], dtype=np.float64).reshape(-1)
            ne_mc_total_ms = (ne_mc_prof["construction_us"] + ne_mc_prof["assemble_and_factor_us"]
                              + ne_mc_prof["solve_us"]) / 1000.0
            res_ne_mc = np.linalg.norm(ata @ x_ne_mc - rhs) / max(1.0, np.linalg.norm(rhs))
        else:
            ne_mc_total_ms = None
            res_ne_mc = None

        if scipy_prof is not None:
            scipy_ms = scipy_prof["total_ms"]
            res_scipy = np.linalg.norm(ata @ x_scipy - rhs) / max(1.0, np.linalg.norm(rhs))
        else:
            scipy_ms = None
            res_scipy = None

        results.append({
            "name": name,
            "shape": f"{m}x{n}",
            "nnz": A.nnz,
            "conex_cliques": conex_cliques,
            "implicit_cliques": implicit_cliques,
            "conex_ms": conex_ms,
            "implicit_ms": implicit_ms,
            "ne_ms": ne_total_ms,
            "ne_mc_ms": ne_mc_total_ms,
            "scipy_ms": scipy_ms,
            "res_conex": res_conex,
            "res_implicit": res_implicit,
            "res_ne": res_ne,
            "res_ne_mc": res_ne_mc,
            "res_scipy": res_scipy,
        })

    # Print table.
    print("\n")
    hdr = (
        f"{'Matrix':<12s} {'Shape':>10s} {'nnz':>6s} "
        f"{'Cliques':>7s} "
        f"{'Conex':>9s} {'Implicit':>9s} {'NE':>9s} {'NE-MC':>9s} {'SciPy':>9s} "
        f"{'Res(conex)':>11s} {'Res(impl)':>11s} {'Res(NE)':>11s} {'Res(MC)':>11s} {'Res(scipy)':>11s}"
    )
    sep = "-" * len(hdr)
    print(sep)
    print(hdr)
    print(sep)
    def fmt_time(v):
        return f"{v:>8.1f}ms" if v is not None else "    FAIL "

    def fmt_res(v):
        return f"{v:>11.1e}" if v is not None else "       FAIL"

    def fmt_int(v, w=7):
        return f"{v:>{w}d}" if v is not None else " " * (w - 4) + "FAIL"

    for r in results:
        print(
            f"{r['name']:<12s} {r['shape']:>10s} {r['nnz']:>6d} "
            f"{fmt_int(r['conex_cliques'])} "
            f"{fmt_time(r['conex_ms'])} {fmt_time(r['implicit_ms'])} "
            f"{fmt_time(r['ne_ms'])} {fmt_time(r['ne_mc_ms'])} {fmt_time(r['scipy_ms'])} "
            f"{fmt_res(r['res_conex'])} {fmt_res(r['res_implicit'])} "
            f"{fmt_res(r['res_ne'])} {fmt_res(r['res_ne_mc'])} {fmt_res(r['res_scipy'])}"
        )
    print(sep)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
