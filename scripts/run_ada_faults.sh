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
fault_link="$object"
opencv_cflags=$(pkg-config --cflags "$package" | sed 's/-I/-isystem /g')
"$compiler" "$@" -c -std=c++17 -Wall -Wextra -Wpedantic -Werror \
    -DOPENCV_CALIB3D_TEST_HOOKS -I"$root/cpp" -I"$core/cpp" $opencv_cflags \
    "$root/cpp/opencv_calib3d_shim.cpp" -o "$object"
if [ "$(uname -s)" = Darwin ]; then
    # Keep Apple C++ exceptions inside a libc++-linked Mach-O image. Linking the
    # object directly into the GNAT executable lets its GNU libstdc++ personality
    # interpose on libc++abi and prevents even our guarded catch from working.
    fault_link="$root/obj/ada-faults/libopencv_calib3d_fault.dylib"
    "$compiler" "$@" -dynamiclib "$object" "$core/lib/libopencv_core_shim.dylib" \
        $(pkg-config --libs "$package") -lc++ \
        -Wl,-install_name,"$fault_link" -o "$fault_link"
fi
cd "$root/tests"
gprbuild -p -P refinement_faults.gpr -XOPENCV_CALIB3D_FAULT_OBJECT="$fault_link" \
    -o "$root/tests/bin/run_refinement_faults" -largs \
    $(pkg-config --libs "$package") "$runtime"
if [ "$(uname -s)" = Darwin ]; then
    # The fault artifact resolves our calls first, but the production project still
    # links its relocatable shim. Supply both runtime directories for this test
    # launch only; preserve any caller-provided search path and fail normally.
    # GPR's relative Ada runtime rpaths also differ for this helper output. Use
    # the Alire-selected compiler's absolute Ada and compiler runtime directories.
    ada_runtime=$(gcc -print-file-name=adalib)
    [ -d "$ada_runtime" ] || { echo 'error: selected GNAT Ada runtime not found' >&2; exit 1; }
    ada_runtime=$(CDPATH= cd -- "$ada_runtime" && pwd)
    compiler_lib=$(CDPATH= cd -- "$(dirname "$(command -v gcc)")/../lib" && pwd)
    DYLD_LIBRARY_PATH="$root/lib:$core/lib:$ada_runtime:$compiler_lib${DYLD_LIBRARY_PATH:+:$DYLD_LIBRARY_PATH}" \
        "$root/tests/bin/run_refinement_faults"
else
    sh "$root/scripts/run_native.sh" "$root/tests/bin/run_refinement_faults"
fi