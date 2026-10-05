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

`X_camera = R * X_world + t`, with `C_world = -R^T * t`. The caller supplies
the metric Cartesian world frame; Translation is never camera position.
Projection publishes a validated temporary matrix atomically; native result
publication is guarded by unique ownership, and Ada copies values before destroying
the temporary result. Test-only exception checkpoints are excluded from production.

Task 002 adds one narrow C export using the existing intrinsics/distortion/pose
records. Iterative refinement stages initial rvec/tvec in local native Mats and
calls `solvePnP` with the extrinsic guess and fixed `SOLVEPNP_ITERATIVE`. Output
validation and exception checkpoints precede publication. Native false publishes
no pose; the Ada wrapper preserves its initial world-to-camera value. This is not
a binding of `solvePnPRefineLM` (not common to the 4.1 baseline).

Reprojection residuals/statistics are Ada values computed in Ada from authoritative
`Project_Points` results. Scaled hypot and scale/ssq accumulation avoid avoidable
overflow/underflow. RMS and maximum are pixels, not covariance/uncertainty or a
proof of globally correct localization. The synthetic example refines local
RANSAC inlier arrays; there is no public selection helper or Features dependency.

The test-only false control and exception controls are excluded from production.
A separate Ada executable links the actual fault-enabled shim artifact (object
on Linux, Apple libc++-isolated test dylib on Darwin) without replacing production
libraries, checking false and all 15 refinement stage/kind
exception combinations through the public wrapper.

Task 003 adds only two private C exports: normalized standard undistortion and
Rodrigues matrix access. Undistortion borrows real Core handles, clones valid
strided input, computes into a local Mat, validates schema/count/finiteness, then
publishes atomically. The private nine-double rotation record is checked by C
sizeof/alignment/offsetof and Ada Size/Alignment/Position plus C-written interchange.
Public matrices, semantic normalized coordinates, directions and world rays are
Ada values. Matrix/vector transforms and scaled bearing normalization are Ada;
Rodrigues remains native. R^T maps camera bearings to world; translation enters
point transforms and the camera-center origin, never direction transforms.
The existing fault helper/dylib architecture is reused for 20 new stage/kind cases.
See `camera-rays-contract.md`; no terrain/Earth model or new dependencies.

Task 004 adds a separate semantic homography C record/options and opaque result,
six private exports, and a limited controlled Ada value estimate. Noncontinuous
Nx1 Float64 C2 inputs are snapshotted before legacy findHomography RANSAC; handles
are never retained. Final native H is validated, then original Float64 points are
independently classified with robust forward hypot instead of publishing the
upstream mask. Four-point direct native behavior is excluded by a >=5 binding
minimum. At least four final-model inliers are required before publication.
Pure-Ada mapping reports projective infinity/overflow without clamping. This is
planar/projective verification, not a 3-D terrain pose substitute or a new backend.
