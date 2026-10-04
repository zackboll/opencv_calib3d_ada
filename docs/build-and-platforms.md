# Build and platforms

`configure_opencv.sh` probes `opencv5`, `opencv4`, then `opencv` pkg-config
metadata. Supported bootstrap range is OpenCV 4.1+ through 4.x and OpenCV 5.0.x.
It fails closed on malformed/unreviewed versions and verifies the backend-specific
header and native shared/import/static library before generating build metadata.

Linux uses the normal GNU C++ runtime and a static-PIC shim. macOS uses Apple
`clang++`/`libc++` and a relocatable shim. Windows uses the MSYS2 MinGW64 OpenCV
package and **MSYS2 MinGW64 g++**, never GNAT's g++, to build an external DLL and
import library.

PR CI: repository checks, Linux, macOS and Linux sanitizers.
Windows: push to `main` only, after merge.
Pinned 4.1/4.10/5.0 source builds: manual workflow only.

Run root, tests and examples Alire operations serially. Local qualification used
Alire 2.1.1, GNAT native 16.1.0, GPRbuild 26.0.1 and OpenCV 4.10.0/calib3d.
The native raw-boundary driver runs on Linux and macOS. The Linux-only sanitizer
mode recompiles the actual production shim and the test-hook variant with ASan
and UBSan, leak detection and halt-on-error, without suppressions. Linked Core
and OpenCV dependencies are not claimed sanitizer-instrumented.
