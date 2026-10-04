# Architecture

`OpenCV.Calib3D` is a handwritten thick binding. The public Ada API is stable
across the supported native module move:

| OpenCV | Native header | Native library | Ada package |
|---|---|---|---|
| 4.1-4.x | `opencv2/calib3d.hpp` | `opencv_calib3d` | `OpenCV.Calib3D` |
| 5.0 | `opencv2/geometry/3d.hpp` | `opencv_geometry` | `OpenCV.Calib3D` |

The OpenCV 5 native module name does not grant this crate ownership of the Ada
`OpenCV.Geometry` namespace.

Core owns every application `Mat`. Public point arrays are marshalled into
Core-owned `Float64 C3`/`C2` Mats using Core typed accessors. Native pointers are
borrowed only inside `OpenCV.Core.Module_Interop` callbacks and are never retained.
The authoritative Core bridge header is used from the resolved dependency; no copy
is stored in this repository.

The C boundary contains only fixed-width/`double` C records, opaque temporary
result handles and Core opaque Mat handles. C++ catches every exception before it
can cross the ABI. RANSAC inliers are validated, sorted and copied to Ada-owned
one-based values.
