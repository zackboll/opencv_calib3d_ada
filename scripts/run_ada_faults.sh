#!/bin/sh
# Run from the tests Alire environment; never replace the production library.
set -eu
root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
core=${OPENCV_CORE_ALIRE_PREFIX:?run using alr -C tests exec}
package=
for candidate in opencv5 opencv4 opencv; do
    if pkg-config --exists "$candidate"; then package=$candidate; break; fi
done
[ -n "$package" ] || { echo 'error: missing OpenCV metadata' >&2; exit 1; }
compiler=${CXX:-g++}
runtime=-lstdc++
set --
if [ "$(uname -s)" = Darwin ]; then
    compiler=$(xcrun --find clang++)
    runtime=-lc++
    set -- -isysroot "$(xcrun --sdk macosx --show-sdk-path)"
fi
mkdir -p "$root/obj/ada-faults"
object="$root/obj/ada-faults/shim.o"
opencv_cflags=$(pkg-config --cflags "$package" | sed 's/-I/-isystem /g')
"$compiler" "$@" -c -std=c++17 -Wall -Wextra -Wpedantic -Werror \
    -DOPENCV_CALIB3D_TEST_HOOKS -I"$root/cpp" -I"$core/cpp" $opencv_cflags \
    "$root/cpp/opencv_calib3d_shim.cpp" -o "$object"
cd "$root/tests"
gprbuild -p -P refinement_faults.gpr -XOPENCV_CALIB3D_FAULT_OBJECT="$object" \
    -o "$root/tests/bin/run_refinement_faults" -largs \
    $(pkg-config --libs "$package") "$runtime"
sh "$root/scripts/run_native.sh" "$root/tests/bin/run_refinement_faults"