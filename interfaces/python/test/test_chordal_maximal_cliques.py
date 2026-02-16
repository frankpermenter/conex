#!/usr/bin/env python3
import unittest

import numpy as np
import scipy.sparse as sp

import conex


def _csr_from_supports(num_vars, supports):
    rows = []
    cols = []
    vals = []
    for r, support in enumerate(supports):
        for c in sorted(set(int(v) for v in support)):
            rows.append(r)
            cols.append(c)
            vals.append(1.0)
    A = sp.csr_matrix((vals, (rows, cols)), shape=(len(supports), num_vars), dtype=np.float64)
    A.sum_duplicates()
    return A


def _to_tuple_set(cliques):
    return {tuple(sorted(int(v) for v in clique)) for clique in cliques if len(clique) > 0}


class ChordalMaximalCliquesTest(unittest.TestCase):
    def test_star_tree_with_permuted_labels(self):
        # Star graph in variable space:
        # center connected to each leaf => maximal cliques are {center, leaf_i}.
        leaves = 7
        n = leaves + 1
        center = n - 1
        supports = [(center, i) for i in range(leaves)]

        # Randomly permute global labels to ensure invariance to variable naming.
        rng = np.random.default_rng(123)
        perm = rng.permutation(n)
        permuted_supports = [tuple(int(perm[v]) for v in s) for s in supports]
        A = _csr_from_supports(n, permuted_supports)

        got = conex.find_maximal_cliques_implicit_csr(
            A.indptr.astype(np.int64, copy=False),
            A.indices.astype(np.int64, copy=False),
            A.data.astype(np.float64, copy=False),
            int(A.shape[0]),
            int(A.shape[1]),
        )
        got_set = _to_tuple_set(got)

        expected_set = {
            tuple(sorted((int(perm[center]), int(perm[i])))) for i in range(leaves)
        }
        self.assertEqual(got_set, expected_set)

    def test_path_graph(self):
        # Path graph 0-1-2-...-(n-1) => maximal cliques are all adjacent pairs.
        n = 10
        supports = [(i, i + 1) for i in range(n - 1)]
        A = _csr_from_supports(n, supports)

        got = conex.find_maximal_cliques_implicit_csr(
            A.indptr.astype(np.int64, copy=False),
            A.indices.astype(np.int64, copy=False),
            A.data.astype(np.float64, copy=False),
            int(A.shape[0]),
            int(A.shape[1]),
        )
        got_set = _to_tuple_set(got)

        expected_set = {tuple((i, i + 1)) for i in range(n - 1)}
        self.assertEqual(got_set, expected_set)


if __name__ == "__main__":
    unittest.main()
