# conex python interface

This directory uses a pure-Python `ctypes` wrapper (`conex.py`) over
`../libconex.so`.

Build/check:
```bash
make
```

Run tests from repo root:
```bash
LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}:$PWD/interfaces" \
PYTHONPATH="${PYTHONPATH:-}:$PWD/interfaces/python" \
python3 interfaces/python/test/run_tests.py
```
