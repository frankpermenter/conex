#!/usr/bin/env python3
"""Compare conex vs SCS on random exponential cone problems.

Reads the problem dumped by benchmark_expcone --dump, solves with SCS,
and prints the comparison.

Usage:
  ./benchmark_expcone --dump 10 6 42 | python3 compare_expcone.py
  # or standalone:
  python3 compare_expcone.py [m] [p] [seed]
"""
import sys
import time
import numpy as np
import scs


def generate_problem(m, p, seed):
    """Generate the same random exp cone problem as the C++ code."""
    rng = np.random.RandomState(seed)
    # Match C++ srand(seed) + MatrixXd::Random behavior:
    # Eigen's Random() uses rand() which gives values in [-1, 1].
    # We'll read from --dump instead for exact match.
    raise NotImplementedError("Use --dump for exact match")


def read_problem_stdin():
    """Read problem from stdin (format from benchmark_expcone --dump)."""
    line1 = input().split()
    m, p = int(line1[0]), int(line1[1])

    c = np.array(input().split(), dtype=float)
    assert len(c) == p

    A_blocks = []
    b_blocks = []
    for i in range(m):
        a_vals = np.array(input().split(), dtype=float)
        assert len(a_vals) == 3 * p
        A_blocks.append(a_vals.reshape(3, p))
        b_vals = np.array(input().split(), dtype=float)
        assert len(b_vals) == 3
        b_blocks.append(b_vals)

    return m, p, c, A_blocks, b_blocks


def solve_scs(m, p, c, A_blocks, b_blocks):
    """Solve with SCS.

    SCS form: min c'x s.t. Ax + s = b, s in K
    Our problem: b_i - A_i x in K_exp
    => -A_i x + s_i = b_i => s_i = b_i - A_i x in K_exp
    So A_scs = -A_stack, b_scs = b_stack.
    """
    n = 3 * m
    A_dense = np.vstack([-Ai for Ai in A_blocks])
    b = np.concatenate(b_blocks)

    import scipy.sparse as sp
    A_sparse = sp.csc_matrix(A_dense)

    cone = {'ep': m}  # m exponential cones

    data = {
        'c': c,
        'A': A_sparse,
        'b': b,
    }

    solver = scs.SCS(data, cone, max_iters=5000, eps_abs=1e-9, eps_rel=1e-9,
                     verbose=False)
    t0 = time.time()
    sol = solver.solve()
    solve_ms = (time.time() - t0) * 1000

    obj = c @ sol['x']
    return obj, sol['info']['iter'], solve_ms, sol['info']['status']


def main():
    m, p, c, A_blocks, b_blocks = read_problem_stdin()
    obj, iters, ms, status = solve_scs(m, p, c, A_blocks, b_blocks)
    print(f"\n=== SCS exp cone: m={m} cones, p={p} vars ===")
    print(f"  Status:     {status}")
    print(f"  Objective:  {obj:.6e}")
    print(f"  Iterations: {iters}")
    print(f"  Solve time: {ms:.1f} ms")


if __name__ == '__main__':
    main()
