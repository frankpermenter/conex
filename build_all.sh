#!/usr/bin/env bash
set -euo pipefail

export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}:$PWD/interfaces"
export LIBRARY_PATH="${LIBRARY_PATH:-}:$PWD/interfaces"
export PYTHONPATH="${PYTHONPATH:-}:$PWD/interfaces/python"

mydir="$(dirname "$(realpath "$0")")"
if [[ "$PWD" != "$mydir" ]]; then
  echo "Error: Script cannot be run from different directory."
  exit 1
fi


bazel_config=debug
bazel_startup_flags=(--output_user_root=/tmp/conex-bazel)
if [[ "${SKIP_FORMATTING:-0}" != "1" ]]; then
  bazel "${bazel_startup_flags[@]}" run //:buildifier --config="$bazel_config"
  find ./ -type f -name "*.h" | xargs clang-format -i
  find ./ -type f -name "*.cc" | xargs clang-format -i
fi


## Build and test repo.
if [[ "${SKIP_BAZEL:-0}" != "1" ]]; then
  bazel "${bazel_startup_flags[@]}" test --config="$bazel_config" //conex/...
fi

## Build and test C API.
make -C interfaces -j 8
if [[ "${SKIP_BAZEL:-0}" != "1" ]]; then
  bazel "${bazel_startup_flags[@]}" test --cache_test_results=no --config="$bazel_config" //interfaces/...
fi

## Build and test Python interface.
make -C interfaces/python clean
make -C interfaces/python
python3 interfaces/python/test/run_tests.py
