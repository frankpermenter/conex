#!/bin/bash
# Build script that handles stale CMake caches from different environments.
# Safe to run from any machine that has the source checkout.
set -e
cd "$(dirname "$0")"

# Check if CMakeCache exists and was generated for a different directory.
if [ -f CMakeCache.txt ]; then
  cached_dir=$(grep 'CMAKE_HOME_DIRECTORY:INTERNAL' CMakeCache.txt 2>/dev/null | cut -d= -f2)
  if [ -n "$cached_dir" ] && [ "$cached_dir" != "$(pwd)" ]; then
    echo "Stale CMake cache (was: $cached_dir, now: $(pwd)). Reconfiguring."
    rm -f CMakeCache.txt cmake_install.cmake Makefile CTestTestfile.cmake
    rm -rf CMakeFiles/ _deps/ Testing/
  fi
fi

cmake -DCMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE:-Release}" . "$@"
make -j"$(nproc)"
