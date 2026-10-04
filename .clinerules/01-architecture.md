# Architecture

- Public Ada namespace: `OpenCV.Calib3D`.
- Native OpenCV 4.x backend: `opencv_calib3d` / `opencv2/calib3d.hpp`.
- Native OpenCV 5.0 backend: `opencv_geometry` / `opencv2/geometry/3d.hpp`.
- Core owns `Mat`; use callback-scoped `OpenCV.Core.Module_Interop` handles.
- Never vendor `opencv_core_module_bridge.hpp`.
- Keep public outputs as Ada values; do not expose native handles.
- C++ exceptions must not cross the C ABI.
