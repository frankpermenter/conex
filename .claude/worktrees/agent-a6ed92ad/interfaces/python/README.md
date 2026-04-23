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
