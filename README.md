# conex

A geodesic interior-point method for conic optimization (LP, QP, SDP, SOCP, exponential cone).

## Build

Requires CMake 3.14+, a C++17 compiler, and Eigen 3.

```bash
mkdir build && cd build
cmake ..
make -j$(nproc)
ctest
```

## Repository Structure

```
src/conex/          C++ library source
  common/           Model, solver, cone ops, constraint assemblers
  algorithms/       Geodesic IPM, HSDE, hybrid, theta-continuation
  tree_solver/      Supernodal Cholesky factorization
tests/              GTest unit tests
bench/              Benchmarks and utilities
doc/                LaTeX documentation
python/             Python bindings (planned)
legacy/             Archived Bazel-based code and old C API
```

## Citation

```
@article{permenter2020geodesic,
  title={A geodesic interior-point method for linear optimization over symmetric cones},
  author={Permenter, Frank},
  year={2020}
}
```
