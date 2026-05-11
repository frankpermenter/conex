#!/usr/bin/env python3
import argparse
import csv
import gzip
import math
import time
from pathlib import Path

import matplotlib
import numpy as np
import scipy.io
import scipy.sparse as sp
import scipy.sparse.linalg as spla

import conex

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


def _find_matrix_market_files(root: Path):
    files = []
    for p in root.rglob("*"):
        if not p.is_file():
            continue
        name = p.name.lower()
        if name.endswith(".mtx") or name.endswith(".mtx.gz"):
            files.append(p)
    return sorted(files)


def _load_matrix_market(path: Path):
    if path.name.lower().endswith(".gz"):
        with gzip.open(path, "rb") as f:
            m = scipy.io.mmread(f)
    else:
        m = scipy.io.mmread(path)
    if not sp.issparse(m):
        m = sp.coo_matrix(m)
    m = m.tocsr()
    if np.iscomplexobj(m.data):
        raise ValueError("complex matrix not supported")
    m.sum_duplicates()
    return m.astype(np.float64, copy=False)


def _generate_synthetic_suite(count, seed, min_rows, max_rows, min_cols, max_cols):
    rng = np.random.default_rng(seed)
    out = []
    for k in range(count):
        n_cols = int(rng.integers(min_cols, max_cols + 1))
        n_rows = int(rng.integers(max(min_rows, n_cols), max_rows + 1))
        density = float(rng.uniform(0.001, 0.02))
        A = sp.random(
            n_rows,
            n_cols,
            density=density,
            format="csr",
            random_state=rng,
            data_rvs=lambda n: rng.normal(0.0, 1.0, size=n),
        )
        A.sum_duplicates()
        out.append((f"synthetic_{k}", A))
    return out


def _matrix_corpus(args):
    corpus = []
    if args.suitesparse_dir is not None:
        root = Path(args.suitesparse_dir)
        if not root.exists():
            raise FileNotFoundError(f"suitesparse_dir does not exist: {root}")
        files = _find_matrix_market_files(root)
        if args.limit_files > 0:
            files = files[: args.limit_files]
        for p in files:
            try:
                A = _load_matrix_market(p)
            except Exception:
                continue
            m, n = A.shape
            if m < args.min_rows or m > args.max_rows:
                continue
            if n < args.min_cols or n > args.max_cols:
                continue
            if A.nnz <= 0:
                continue
            corpus.append((str(p), A))
            if len(corpus) >= args.max_matrices:
                break
    if not corpus:
        corpus = _generate_synthetic_suite(
            count=args.synthetic_count,
            seed=args.seed,
            min_rows=args.min_rows,
            max_rows=args.max_rows,
            min_cols=args.min_cols,
            max_cols=args.max_cols,
        )
    return corpus


def _row_support_groups(Acsr):
    groups = {}
    for r in range(Acsr.shape[0]):
        lo = Acsr.indptr[r]
        hi = Acsr.indptr[r + 1]
        support = tuple(sorted(set(int(c) for c in Acsr.indices[lo:hi])))
        if not support:
            continue
        groups.setdefault(support, []).append(r)
    return groups


def _maximal_chordal_cover_bags(Acsr):
    if Acsr.shape[0] == 0 or Acsr.shape[1] == 0:
        return []
    raw = conex.find_maximal_cliques_implicit_csr(
        Acsr.indptr.astype(np.int64, copy=False),
        Acsr.indices.astype(np.int64, copy=False),
        Acsr.data.astype(np.float64, copy=False),
        int(Acsr.shape[0]),
        int(Acsr.shape[1]),
    )
    bags = [tuple(sorted(int(v) for v in bag)) for bag in raw if len(bag) > 0]
    covered = set(v for bag in bags for v in bag)
    for v in range(Acsr.shape[1]):
        if v not in covered:
            bags.append((int(v),))
    unique = []
    for b in bags:
        if b not in unique:
            unique.append(b)
    sets = [set(b) for b in unique]
    maximal = []
    for i, b in enumerate(unique):
        sb = sets[i]
        is_maximal = True
        for j in range(len(unique)):
            if i == j:
                continue
            if len(unique[i]) < len(unique[j]) and sb.issubset(sets[j]):
                is_maximal = False
                break
        if is_maximal:
            maximal.append(b)
    return maximal


def _build_support_block(Acsr, rows, support):
    vars_ = list(support)
    ag = Acsr[rows, :][:, vars_].toarray()
    return ag.T @ ag


def _solve_conex_support_groups(Acsr, b, reg, num_threads, clique_tree_method):
    groups = _row_support_groups(Acsr)
    labels = []
    submatrices = []
    for support, rows in groups.items():
        block = _build_support_block(Acsr, rows, support)
        labels.append(support)
        submatrices.append([block])
    for j in range(Acsr.shape[1]):
        labels.append((j,))
        submatrices.append([np.array([[reg]], dtype=np.float64)])
    rhs = Acsr.T @ b
    x = conex.sparse_solve_blocks_tree(
        submatrices_by_group=submatrices,
        labels=labels,
        b=rhs,
        num_threads=num_threads,
        clique_tree_method=clique_tree_method,
    )
    return np.asarray(x, dtype=np.float64).reshape(-1)


def _solve_conex_maximal_chordal(Acsr, b, reg, num_threads, clique_tree_method):
    # Intentionally uses the C++ implicit-clique path to avoid Python-side
    # bag/block construction overhead.
    x = conex.sparse_ls_csr_implicit(
        Acsr.indptr.astype(np.int64, copy=False),
        Acsr.indices.astype(np.int64, copy=False),
        Acsr.data.astype(np.float64, copy=False),
        int(Acsr.shape[0]),
        int(Acsr.shape[1]),
        np.asarray(b, dtype=np.float64),
        int(num_threads),
    )
    return np.asarray(x, dtype=np.float64).reshape(-1)


def _solve_scipy_sparse(Acsr, b, reg):
    ata = Acsr.T @ Acsr
    ata = ata + reg * sp.eye(Acsr.shape[1], format="csr")
    rhs = Acsr.T @ b
    return spla.spsolve(ata, rhs)


def _solve_numpy_dense(Acsr, b, reg):
    ata = (Acsr.T @ Acsr).toarray()
    ata[np.diag_indices_from(ata)] += reg
    rhs = Acsr.T @ b
    return np.linalg.solve(ata, rhs)


def _time_solver(fn, reps):
    times = []
    out = None
    for _ in range(reps):
        t0 = time.perf_counter()
        out = fn()
        times.append((time.perf_counter() - t0) * 1000.0)
    times.sort()
    return times[len(times) // 2], out


def _performance_profile_rows(results, solvers):
    rows = {s: [] for s in solvers}
    for r in results:
        if any(s not in r["times_ms"] for s in solvers):
            continue
        best = min(r["times_ms"][s] for s in solvers)
        for s in solvers:
            rows[s].append(r["times_ms"][s] / max(1e-12, best))
    return rows


def _plot_performance_profile(profile_rows, out_path: Path):
    taus = np.logspace(0.0, 2.0, 300)
    fig = plt.figure(figsize=(7, 4.5), dpi=150)
    ax = fig.add_subplot(111)
    for solver, ratios in profile_rows.items():
        ratios = np.asarray(ratios, dtype=np.float64)
        ys = np.asarray([(ratios <= t).mean() for t in taus], dtype=np.float64)
        ax.plot(taus, ys, label=solver, linewidth=2)
    ax.set_xscale("log")
    ax.set_ylim(0.0, 1.02)
    ax.set_xlabel("tau")
    ax.set_ylabel("rho_s(tau)")
    ax.set_title("Dolan-More Performance Profile")
    ax.grid(True, which="both", alpha=0.2)
    ax.legend(loc="lower right")
    fig.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Benchmark least-squares normal-equation solvers with Dolan-More "
            "performance profiles. Uses maximal cliques from a min-fill chordal cover "
            "for the Conex 'best config' path."
        )
    )
    parser.add_argument("--suitesparse-dir", type=str, default=None)
    parser.add_argument("--max-matrices", type=int, default=20)
    parser.add_argument("--limit-files", type=int, default=0)
    parser.add_argument("--synthetic-count", type=int, default=10)
    parser.add_argument("--min-rows", type=int, default=200)
    parser.add_argument("--max-rows", type=int, default=4000)
    parser.add_argument("--min-cols", type=int, default=100)
    parser.add_argument("--max-cols", type=int, default=2000)
    parser.add_argument("--reps", type=int, default=3)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--reg", type=float, default=1e-8)
    parser.add_argument("--num-threads", type=int, default=8)
    parser.add_argument(
        "--clique-tree-method",
        type=int,
        default=int(conex.CLIQUE_TREE_METHOD_AMD),
    )
    parser.add_argument("--dense-max-cols", type=int, default=1200)
    parser.add_argument(
        "--out-csv",
        type=str,
        default="interfaces/python/test/benchmark_outputs/suitesparse_ls_results.csv",
    )
    parser.add_argument(
        "--out-profile-png",
        type=str,
        default="interfaces/python/test/benchmark_outputs/suitesparse_ls_perf_profile.png",
    )
    args = parser.parse_args()

    rng = np.random.default_rng(args.seed)
    corpus = _matrix_corpus(args)
    results = []

    print(
        "name,m,n,nnz,solver_conex_maximal_ms,solver_conex_support_ms,"
        "solver_scipy_sparse_ms,solver_numpy_dense_ms,"
        "relerr_maximal_vs_sparse,relerr_support_vs_sparse,"
        "res_maximal,res_support,res_sparse"
    )

    for name, A in corpus:
        Acsr = A.tocsr(copy=True)
        m, n = Acsr.shape
        x_true = rng.normal(0.0, 1.0, size=n)
        b = Acsr @ x_true

        solvers = {
            "conex_maximal_chordal": lambda: _solve_conex_maximal_chordal(
                Acsr,
                b,
                args.reg,
                args.num_threads,
                args.clique_tree_method,
            ),
            "conex_support_groups": lambda: _solve_conex_support_groups(
                Acsr,
                b,
                args.reg,
                args.num_threads,
                args.clique_tree_method,
            ),
            "scipy_sparse": lambda: _solve_scipy_sparse(Acsr, b, args.reg),
        }
        if n <= args.dense_max_cols:
            solvers["numpy_dense"] = lambda: _solve_numpy_dense(Acsr, b, args.reg)

        times = {}
        outputs = {}
        for key, fn in solvers.items():
            try:
                # Warmup once.
                _ = fn()
                t_ms, x = _time_solver(fn, args.reps)
                times[key] = t_ms
                outputs[key] = np.asarray(x, dtype=np.float64).reshape(-1)
            except Exception as exc:
                print(f"# WARN case={name} solver={key} error={exc}")
                continue

        if "scipy_sparse" not in outputs:
            print(f"# WARN case={name} skipped: scipy_sparse failed.")
            continue

        x_ref = outputs["scipy_sparse"]
        K = (Acsr.T @ Acsr) + args.reg * sp.eye(n, format="csr")
        rhs = Acsr.T @ b
        if "conex_maximal_chordal" in outputs:
            rel_max = np.linalg.norm(outputs["conex_maximal_chordal"] - x_ref) / max(
                1.0, np.linalg.norm(x_ref)
            )
            res_max = np.linalg.norm(K @ outputs["conex_maximal_chordal"] - rhs) / max(
                1.0, np.linalg.norm(rhs)
            )
        else:
            rel_max = math.nan
            res_max = math.nan
        if "conex_support_groups" in outputs:
            rel_sup = np.linalg.norm(outputs["conex_support_groups"] - x_ref) / max(
                1.0, np.linalg.norm(x_ref)
            )
            res_sup = np.linalg.norm(K @ outputs["conex_support_groups"] - rhs) / max(
                1.0, np.linalg.norm(rhs)
            )
        else:
            rel_sup = math.nan
            res_sup = math.nan
        res_ref = np.linalg.norm(K @ x_ref - rhs) / max(1.0, np.linalg.norm(rhs))

        record = {
            "name": name,
            "m": int(m),
            "n": int(n),
            "nnz": int(Acsr.nnz),
            "times_ms": times,
            "relerr_maximal_vs_sparse": float(rel_max),
            "relerr_support_vs_sparse": float(rel_sup),
            "res_maximal": float(res_max),
            "res_support": float(res_sup),
            "res_sparse": float(res_ref),
        }
        results.append(record)

        print(
            f"{name},{m},{n},{Acsr.nnz},"
            f"{times.get('conex_maximal_chordal', float('nan')):.3f},"
            f"{times.get('conex_support_groups', float('nan')):.3f},"
            f"{times.get('scipy_sparse', float('nan')):.3f},"
            f"{times.get('numpy_dense', float('nan')):.3f},"
            f"{rel_max:.3e},{rel_sup:.3e},{res_max:.3e},{res_sup:.3e},{res_ref:.3e}"
        )

    if not results:
        raise RuntimeError("No benchmark cases ran.")

    out_csv = Path(args.out_csv)
    out_csv.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "name",
        "m",
        "n",
        "nnz",
        "conex_maximal_chordal_ms",
        "conex_support_groups_ms",
        "scipy_sparse_ms",
        "numpy_dense_ms",
        "relerr_maximal_vs_sparse",
        "relerr_support_vs_sparse",
        "res_maximal",
        "res_support",
        "res_sparse",
    ]
    with out_csv.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        for r in results:
            w.writerow(
                {
                    "name": r["name"],
                    "m": r["m"],
                    "n": r["n"],
                    "nnz": r["nnz"],
                    "conex_maximal_chordal_ms": r["times_ms"].get("conex_maximal_chordal"),
                    "conex_support_groups_ms": r["times_ms"].get("conex_support_groups"),
                    "scipy_sparse_ms": r["times_ms"].get("scipy_sparse"),
                    "numpy_dense_ms": r["times_ms"].get("numpy_dense", math.nan),
                    "relerr_maximal_vs_sparse": r["relerr_maximal_vs_sparse"],
                    "relerr_support_vs_sparse": r["relerr_support_vs_sparse"],
                    "res_maximal": r["res_maximal"],
                    "res_support": r["res_support"],
                    "res_sparse": r["res_sparse"],
                }
            )

    perf_rows = _performance_profile_rows(
        results, ["conex_maximal_chordal", "conex_support_groups", "scipy_sparse"]
    )
    if all(len(v) > 0 for v in perf_rows.values()):
        _plot_performance_profile(perf_rows, Path(args.out_profile_png))
    else:
        print("# WARN not enough successful rows for full 3-solver performance profile.")

    def gmean(values):
        vals = [max(1e-12, float(v)) for v in values]
        return float(np.exp(np.mean(np.log(vals))))

    ratio_max_vs_sparse = [
        r["times_ms"]["conex_maximal_chordal"] / max(1e-12, r["times_ms"]["scipy_sparse"])
        for r in results
        if "conex_maximal_chordal" in r["times_ms"] and "scipy_sparse" in r["times_ms"]
    ]
    ratio_support_vs_sparse = [
        r["times_ms"]["conex_support_groups"] / max(1e-12, r["times_ms"]["scipy_sparse"])
        for r in results
        if "conex_support_groups" in r["times_ms"] and "scipy_sparse" in r["times_ms"]
    ]
    wins_max = sum(x < 1.0 for x in ratio_max_vs_sparse)
    wins_sup = sum(x < 1.0 for x in ratio_support_vs_sparse)
    print(
        f"SUMMARY cases={len(results)} "
        f"gmean(conex_maximal/scipy_sparse)={gmean(ratio_max_vs_sparse) if ratio_max_vs_sparse else math.nan:.3f} "
        f"gmean(conex_support/scipy_sparse)={gmean(ratio_support_vs_sparse) if ratio_support_vs_sparse else math.nan:.3f} "
        f"wins_maximal={wins_max}/{len(ratio_max_vs_sparse)} "
        f"wins_support={wins_sup}/{len(ratio_support_vs_sparse)} "
        f"profile_png={args.out_profile_png} csv={args.out_csv}"
    )


if __name__ == "__main__":
    main()
