#!/usr/bin/env bash
set -euo pipefail

buildifier_bin="$1"
shift

if [[ "$#" -eq 0 ]]; then
  exec "${buildifier_bin}" -r .
fi

exec "${buildifier_bin}" "$@"
