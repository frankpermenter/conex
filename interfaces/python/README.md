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
