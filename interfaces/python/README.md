# conex python interface

This directory uses a C++ `pybind11` extension module (`_conex`) plus a small
Python shim (`conex.py`).

Build the extension:
```bash
make -C interfaces/python
```

Build with pip/scikit-build-core:
```bash
python3 -m pip install -e interfaces/python
```

Run tests from repo root:
```bash
LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}:$PWD/interfaces" \
PYTHONPATH="${PYTHONPATH:-}:$PWD/interfaces/python" \
python3 interfaces/python/test/run_tests.py
```

Sparse least-squares helper:
```python
import conex
import numpy as np
import scipy.sparse as sp

A = sp.random(100, 40, density=0.05, format="csr")
b = A @ (0.1 * np.arange(40))
x = conex.sparse_ls(A, b, num_threads=4)
```

Experimental tree-decomposition-based sparse least-squares helper (built on
the exposed `KKTTreeSolver` API):

```python
x = conex.sparse_ls_tree(A, b, num_threads=4)
```

`sparse_ls_tree` uses an off-the-shelf min-fill tree decomposition heuristic
(`networkx`) to build a clique tree. If `networkx` is unavailable, it falls
back to `conex.sparse_ls`.

Labeled block-assembly solver interface:

```python
x = conex.sparse_solve_blocks_tree(
    submatrices_by_group=[[B0], [B1a, B1b]],
    labels=[(0, 1), (1, 2)],
    b=b,
    num_threads=4,
)
```

Each `labels[i]` tuple gives global variable labels for all blocks in
`submatrices_by_group[i]`; blocks in the same group are summed before solve.
The routine builds a variable support graph from the label tuples, computes a
min-fill tree decomposition, and assigns each labeled block to a decomposition
clique that is a superset of its support. This gives a running-intersection
compatible clique tree and explicit fill-in via the decomposition.

Benchmark Conex tree least-squares against SciPy/NumPy:
```bash
LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}:$PWD/interfaces" \
PYTHONPATH="${PYTHONPATH:-}:$PWD/interfaces/python" \
python3 interfaces/python/test/sparse_ls_benchmark.py
```

The low-level solver config also exposes `num_threads`:
```python
cfg = conex.CONEX_SolverConfiguration()
conex.CONEX_SetDefaultOptions(cfg)
cfg.num_threads = 4
```
