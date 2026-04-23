# conex
An implementation of a geodesic interior-point method for symmetric cone optimization.

## Build requirements

- Bazel **9.0.0** is required for Bazel targets in this repo (pinned in `.bazelversion`).
- Recommended: install and use `bazelisk` so the version in `.bazelversion` is selected automatically.
- If you use a plain `bazel` binary directly, it should be version `9.0.0`.

## Quick start

Run core tests:
```bash
bazel test --config=debug //conex/...
```

Run the full project build/test script:
```bash
./build_all.sh
```

Useful flags for `build_all.sh`:
- `SKIP_FORMATTING=1` to skip `buildifier` and `clang-format`
- `SKIP_BAZEL=1` to skip Bazel test steps

If this code is useful to you, please cite:
```
@article{permenter2020geodesic,
  title={A geodesic interior-point method for linear optimization over symmetric cones},
  author={Permenter, Frank},
  year={2020}
}
