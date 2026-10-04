#!/bin/sh
set -eu
root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
cd "$root"
alr -n build
alr -n -C tests build
alr -n -C tests exec -- sh ../scripts/run_native.sh bin/run_tests
if [ "$(uname -s)" = Linux ]; then
    alr -n exec -- sh scripts/run_sanitizers.sh native
fi
