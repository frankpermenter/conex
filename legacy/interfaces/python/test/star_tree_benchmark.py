#!/usr/bin/env python3
import time
from pathlib import Path

import matplotlib
import numpy as np
import scipy.sparse as sp
import scipy.sparse.linalg as spla

from _conex import sparse_ls_profile_csr

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


def build_star_matrix_expected_tree(
    separator_size: int, num_leaves: int, repeats: int, seed: int
) -> sp.csr_matrix:
    # Variables: [leaf blocks | separator block], with leaf block size = separator_size.
    # Unique supports are exactly:
    #  - one separator support of size separator_size
    #  - one support per leaf of size 2 * separator_size
    # Therefore expected cliques = 1 + num_leaves.
    leaf_block_size = separator_size
    n = separator_size + num_leaves * leaf_block_size
    rows = []
    cols = []
    vals = []
    r = 0
    rng = np.random.default_rng(seed)

    separator_start = num_leaves * leaf_block_size
    separator_cols = list(range(separator_start, separator_start + separator_size))

    # Separator-only rows with full separator support (single unique support).
    for _ in range(separator_size):
        coeff = rng.normal(0.0, 0.25, size=separator_size)
        for j, c in zip(separator_cols, coeff):
            rows.append(r)
            cols.append(j)
            vals.append(float(c))
        r += 1

    # Per-leaf rows with support = full separator block + current leaf block.
    # repeats+1 keeps leaf-block conditioning while preserving support pattern.
    rows_per_leaf = max(repeats + 1, leaf_block_size)
    for leaf in range(num_leaves):
        leaf_start = leaf * leaf_block_size
        leaf_cols = list(range(leaf_start, leaf_start + leaf_block_size))
        for rep in range(rows_per_leaf):
            coeff = rng.normal(0.0, 0.25, size=separator_size)
            for j, c in zip(separator_cols, coeff):
                rows.append(r)
                cols.append(j)
                vals.append(float(c))
            # Keep all columns in support, but cycle a strong coordinate to
            # guarantee full-rank information in each leaf block.
            leaf_coeff = rng.normal(0.0, 0.02, size=leaf_block_size)
            ridge = 1.0 + 0.02 * (rep % 5)
            leaf_coeff[rep % leaf_block_size] += ridge
            for j, c in zip(leaf_cols, leaf_coeff):
                rows.append(r)
                cols.append(j)
                vals.append(float(c))
            r += 1

    return sp.csr_matrix((vals, (rows, cols)), shape=(r, n))


def time_median(fn, reps=5):
    times = []
    out = None
    for _ in range(reps):
        t0 = time.perf_counter()
        out = fn()
        times.append((time.perf_counter() - t0) * 1000.0)
    times.sort()
    return times[len(times) // 2], out


def rel_error(x, y):
    denom = max(1.0, np.linalg.norm(y))
    return np.linalg.norm(x - y) / denom


def save_sparsity_pattern(ata: sp.csc_matrix, output_path: Path) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig = plt.figure(figsize=(5.5, 5.5), dpi=150)
    ax = fig.add_subplot(111)
    ax.spy(ata, markersize=0.8, color="black")
    ax.set_title("Sparsity of A^T A")
    ax.set_xlabel("column")
    ax.set_ylabel("row")
    fig.tight_layout()
    fig.savefig(output_path)
    plt.close(fig)


def main():
    instances = []
    for sep in [1, 4, 8, 16, 32]:
        for leaves in [64, 128, 256]:
            instances.append((sep, leaves, 8))
    output_dir = Path("interfaces/python/test/benchmark_outputs")

    print(
        "sep,leaves,expected_cliques,actual_cliques,match,"
        "n,m,nnzA,nnzAtA,tree_total_ms,tree_factor_ms,tree_solve_ms,"
        "scipy_sparse_ms,numpy_dense_ms,sparse/tree,dense/tree,"
        "err_tree_sparse,res_tree,res_sparse,res_dense"
    )

    for idx, (sep, leaves, repeats) in enumerate(instances):
        A = build_star_matrix_expected_tree(sep, leaves, repeats, seed=idx + 1000)
        n = A.shape[1]
        x_true = np.linspace(-1.0, 1.0, n)
        b = A @ x_true
        ata = (A.T @ A).tocsc()
        rhs = A.T @ b

        pattern_path = output_dir / f"ata_pattern_sep{sep}_leaves{leaves}.png"
        save_sparsity_pattern(ata, pattern_path)

        # Warmup each path.
        _ = sparse_ls_profile_csr(
            A.indptr.astype(np.int64, copy=False),
            A.indices.astype(np.int64, copy=False),
            A.data.astype(np.float64, copy=False),
            int(A.shape[0]),
            int(A.shape[1]),
            np.asarray(b, dtype=np.float64),
            8,
        )
        _ = spla.spsolve(ata, rhs)
        _ = np.linalg.solve(ata.toarray(), rhs)

        def run_tree():
            return sparse_ls_profile_csr(
                A.indptr.astype(np.int64, copy=False),
                A.indices.astype(np.int64, copy=False),
                A.data.astype(np.float64, copy=False),
                int(A.shape[0]),
                int(A.shape[1]),
                np.asarray(b, dtype=np.float64),
                8,
            )

        _, prof = time_median(run_tree, reps=5)
        tree_total_ms = float(prof["total_ms"])
        tree_factor_ms = float(prof["factor_ms"])
        tree_solve_ms = float(prof["solve_ms"])
        x_tree = np.asarray(prof["x"], dtype=np.float64)

        sparse_ms, x_sparse = time_median(lambda: spla.spsolve(ata, rhs), reps=5)
        dense_ms, x_dense = time_median(
            lambda: np.linalg.solve(ata.toarray(), rhs), reps=5
        )

        expected = 1 + leaves
        actual = int(prof["num_cliques"])
        matches = int(actual == expected)

        res_tree = np.linalg.norm(A @ x_tree - b)
        res_sparse = np.linalg.norm(A @ x_sparse - b)
        res_dense = np.linalg.norm(A @ x_dense - b)

        print(
            f"{sep},{leaves},{expected},{actual},{matches},"
            f"{A.shape[1]},{A.shape[0]},{A.nnz},{ata.nnz},"
            f"{tree_total_ms:.3f},{tree_factor_ms:.3f},{tree_solve_ms:.3f},"
            f"{sparse_ms:.3f},{dense_ms:.3f},"
            f"{sparse_ms/max(1e-12, tree_total_ms):.3f},"
            f"{dense_ms/max(1e-12, tree_total_ms):.3f},"
            f"{rel_error(x_tree, x_sparse):.3e},"
            f"{res_tree:.3e},{res_sparse:.3e},{res_dense:.3e}"
        )


if __name__ == "__main__":
    main()
