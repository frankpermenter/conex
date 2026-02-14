#!/usr/bin/env python3
import time

import numpy as np
import scipy.sparse as sp
import scipy.sparse.linalg as spla

from _conex import sparse_ls_profile_csr


def build_star_matrix_expected_tree(
    separator_size: int, num_leaves: int, repeats: int, seed: int
) -> sp.csr_matrix:
    # Variables: [separator block | leaf variables]
    # Unique supports are exactly:
    #  - one separator support of size separator_size
    #  - one support per leaf of size separator_size + 1
    # Therefore expected cliques = 1 + num_leaves.
    n = separator_size + num_leaves
    rows = []
    cols = []
    vals = []
    r = 0
    rng = np.random.default_rng(seed)

    # Separator-only rows with full separator support (single unique support).
    for _ in range(separator_size):
        coeff = rng.normal(0.0, 0.25, size=separator_size)
        for j, c in enumerate(coeff):
            rows.append(r)
            cols.append(j)
            vals.append(float(c))
        r += 1

    # Per-leaf rows with support = full separator block + current leaf.
    # repeats+1 keeps leaf diagonal conditioning while preserving support pattern.
    for leaf in range(num_leaves):
        leaf_col = separator_size + leaf
        for rep in range(repeats + 1):
            coeff = rng.normal(0.0, 0.25, size=separator_size)
            for j, c in enumerate(coeff):
                rows.append(r)
                cols.append(j)
                vals.append(float(c))
            rows.append(r)
            cols.append(leaf_col)
            vals.append(0.8 + 0.05 * (rep % 4))
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


def main():
    instances = []
    for sep in [1, 4, 8, 16, 32]:
        for leaves in [64, 128, 256]:
            instances.append((sep, leaves, 8))

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
