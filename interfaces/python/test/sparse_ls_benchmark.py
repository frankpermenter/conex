#!/usr/bin/env python3
import time

import numpy as np
import scipy.sparse as sp
import scipy.sparse.linalg as spla

import conex


def build_star_ls_matrix(num_leaves: int, repeats: int) -> sp.csr_matrix:
    # Variable 0 is the center. Variables 1..num_leaves are leaves.
    n = num_leaves + 1
    rows = []
    cols = []
    vals = []
    r = 0

    # Center diagonal support.
    rows.append(r)
    cols.append(0)
    vals.append(2.0)
    r += 1

    # Rows that only couple center with one leaf. This enforces star sparsity in A^T A.
    for rep in range(repeats):
        scale = 0.7 + 0.2 * (rep % 3)
        for leaf in range(1, n):
            rows.extend([r, r])
            cols.extend([0, leaf])
            vals.extend([1.0, scale])
            r += 1

    # Leaf diagonals for conditioning.
    for leaf in range(1, n):
        rows.append(r)
        cols.append(leaf)
        vals.append(1.0 + 0.01 * (leaf % 11))
        r += 1

    return sp.csr_matrix((vals, (rows, cols)), shape=(r, n))


def rel_error(x, y):
    denom = max(1.0, np.linalg.norm(y))
    return np.linalg.norm(x - y) / denom


def residual_norm(A, x, b):
    return np.linalg.norm(A @ x - b)


def time_call(fn, repeats=3):
    best_ms = float("inf")
    out = None
    for _ in range(repeats):
        t0 = time.perf_counter()
        out = fn()
        dt_ms = (time.perf_counter() - t0) * 1000.0
        best_ms = min(best_ms, dt_ms)
    return best_ms, out


def offdiag_leaf_nnz(ata: sp.csr_matrix) -> int:
    leaf_block = ata[1:, 1:].tocsr(copy=True)
    leaf_block.setdiag(0.0)
    leaf_block.eliminate_zeros()
    return int(leaf_block.nnz)


def main():
    instances = [
        (64, 6),
        (128, 8),
        (256, 10),
        (384, 12),
    ]
    print(
        "n,m,nnzA,nnzAtA,offdiag_leaf_nnz,tree_ms,lsqr_ms,dense_ms,"
        "lsqr/tree,dense/tree,err_tree_lsqr,err_dense_lsqr,res_tree,res_lsqr,res_dense"
    )
    for leaves, repeats in instances:
        A = build_star_ls_matrix(leaves, repeats)
        n = A.shape[1]
        x_true = np.linspace(-0.8, 1.2, n)
        b = A @ x_true
        ata = (A.T @ A).tocsr()
        rhs = A.T @ b

        tree_ms, x_tree = time_call(
            lambda: np.asarray(
                conex.sparse_ls(A, b, num_threads=8),
                dtype=np.float64,
            ),
            repeats=3,
        )
        lsqr_ms, x_lsqr = time_call(
            lambda: spla.lsqr(A, b, atol=1e-10, btol=1e-10, iter_lim=4000)[0],
            repeats=3,
        )
        dense_ms, x_dense = time_call(
            lambda: np.linalg.solve(ata.toarray(), rhs),
            repeats=3,
        )

        print(
            f"{n},{A.shape[0]},{A.nnz},{ata.nnz},{offdiag_leaf_nnz(ata)},"
            f"{tree_ms:.3f},{lsqr_ms:.3f},{dense_ms:.3f},"
            f"{lsqr_ms/tree_ms:.3f},{dense_ms/tree_ms:.3f},"
            f"{rel_error(x_tree, x_lsqr):.3e},{rel_error(x_dense, x_lsqr):.3e},"
            f"{residual_norm(A, x_tree, b):.3e},{residual_norm(A, x_lsqr, b):.3e},"
            f"{residual_norm(A, x_dense, b):.3e}"
        )


if __name__ == "__main__":
    main()
