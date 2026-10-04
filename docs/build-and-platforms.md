# Build and platforms

`configure_opencv.sh` probes `opencv5`, `opencv4`, then `opencv` pkg-config
metadata. Supported bootstrap range is OpenCV 4.1+ through 4.x and OpenCV 5.0.x.
It verifies the backend-specific header before generating build metadata.

Linux uses the normal GNU C++ runtime and a static-PIC shim. macOS uses Apple
`clang++`/`libc++` and a relocatable shim. Windows uses the MSYS2 MinGW64 OpenCV
package and **MSYS2 MinGW64 g++**, never GNAT's g++, to build an external DLL and
import library.

PR CI: repository checks, Linux, macOS and Linux sanitizers.
Windows: push to `main` only, after merge.
Pinned 4.1/4.10/5.0 source builds: manual workflow only.
