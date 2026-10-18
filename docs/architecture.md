# Architecture

## Task 021 landmark relocalization

Ada-only composition of the existing fixed-EPNP RANSAC, owned pose/inlier copy,
inlier-only iterative refinement, native Rodrigues and native projection.
The existing AP3P reference evaluator is reused as a general one-pose diagnostic
engine; **no AP3P solver is called**. Whole-set guards prevent partial scores from
being accepted. Separate fitting/held-out world sets are validated for exact
disjointness. The result owns values independently of Pose_Estimate finalization.
The optional Task 020 adapter consumes only guarded accepted world coordinates.
No C++ implementation, raw pointer, ABI operation (48 unchanged), or dependency
is added. Native errors abort publication; caller policy gates pose and center.
See [contract](landmark-relocalization-contract.md).

## Task 020 metric point refinement

One initial `Triangulate_World_Points` batch is preserved verbatim. Eligible
source values feed the existing pure-Ada `Refine_Triangulated_Points` batch.
Shared internal metric conversion/projection validation serves both Task 019 and
improved candidates, preserving initial behavior and exception boundaries.
Normalized diagnostics of actual representable world coordinates are recomputed
after conversion. Native projection exceptions abort the complete batch, while
arithmetic unavailability remains per point. Private owned results distinguish
original, optimization, candidate diagnostics, and accepted world position.
The fixed-pose chain is unit-baseline refinement -> known baseline scaling ->
metric first-camera coordinates -> world transform -> final pixel/quality policy.
No native ABI operation, new optimizer, C++ change, public handle or dependency.
Pose, baseline, intrinsics and distortion stay fixed; no covariance, depth
uncertainty, bundle adjustment, pose optimization, DTED/geographic conversion or
navigation fusion. See [contract](metric-point-refinement-contract.md).

## Task 019 metric triangulation orchestration

Separate native undistortion for each camera feeds existing native normalized
triangulation. Task 018 supplies relative pose and scaled physical baseline.
Ada scales only Usable source points and shares the existing camera-to-world
arithmetic, with native rotation obtained outside numerical-failure handlers.
Existing quality assessment and two native calibrated projections produce
caller-policy diagnostics. Native exceptions abort the batch; per-point Ada
conversion failures preserve source status without invented coordinates.
Public outputs are owned Ada values with discriminated availability. Core owns
all temporary Mats; the private ABI remains 48 and C++ is unchanged. See the
[contract](metric-triangulation-contract.md).


## Task 017 AP3P refinement

No new C ABI operation. The private ABI remains 48 imports and exports.
`Select_Refine_AP3P_Pose` calls `Select_AP3P_Pose` once. On a unique owned
pose it evaluates the held-out set, calls `Refine_Pose_Iterative` once on the
seeds plus selection references, and publishes the refined pose only when the
caller's held-out policy accepts it. Held-out points do not enter native
AP3P, selection, or refinement. See
[refinement contract](ap3p-refinement-contract.md).

## Task 016 AP3P selection

No new C ABI operation. The private ABI remains 48 imports and exports.
`Select_AP3P_Hypothesis` consumes `Summarize_AP3P_References` and applies the
caller's inclusive thresholds and strict RMS separation in Ada.
`Select_AP3P_Pose` calls the existing solver once, evaluates references, and copies the selected pose into a private result. The matrix result does not own a pose.
A native projection failure still propagates as `OpenCV_Error`. See
[selection contract](ap3p-selection-contract.md).

## Task 015 guarded native epipolar correction

`Correct_Epipolar_Matches` calls native `cv::correctMatches` exactly once per
nonempty request through one new guarded private operation,
`opencv_calib3d_correct_matches`; the private ABI is now **48** imports and
exports. The shim validates options, normalizes F by its maximum absolute
coefficient (rejecting silent coefficient underflow), measures native Float64 SVD
ratios against caller policy, screens input epipolar-line quality, snapshots
strided Nx1 inputs into contiguous 1xN matrices (the legacy 4.x layout), and
verifies native output (schema, finiteness, corrected residual, one-sided
displacement bound) before publishing one Nx2 CV_64FC2 result. The pure checks
live in `cpp/correction_profile.hpp`, which contains no correction algorithm.
Ada independently validates, re-verifies every published pair and computes
diagnostics; it never solves for corrected points. Checkpoints 58..61 and a
test-only post-native output control exist only in fault builds. See the
[contract](epipolar-correction-contract.md).

## Task 014 native pose composition

`Compose_Poses (First, Second)` calls native `cv::composeRT` (no Jacobians)
through one new guarded private operation. The private ABI is now 47 imports
and exports. Ada validates, marshals and checks results; it does not compose
poses. Aliased output is rejected before clearing. See the
[contract](pose-composition-contract.md), [source review](pose-composition-source-review.md)
and [qualification](pose-composition-validation.md).

## Task 018 relative camera motion

No new C ABI operation. Inversion, relative pose, and baseline diagnostics reuse native Rodrigues, camera-center, and `composeRT`. The private ABI remains 48 operations. See the [contract](relative-camera-motion-contract.md).

## Task 013 reference diagnostics

No new C ABI operation. The private ABI remains 46 imports and exports.
`Evaluate_AP3P_References` borrows `Rotation_Matrix_Of` once per hypothesis
and calls the existing `Project_Points` binding only for references whose
camera-frame `Z` is strictly positive. Pixel error reuses the scaled
Euclidean residual established by `Reprojection_Errors`. Summaries call
`Summarize_Reprojection`. Ada does not implement the Brown polynomial.
See [reference contract](ap3p-reference-contract.md).

## Task 012 AP3P boundary

One additional guarded C operation brings the private ABI to 46 operations.
The caller-owned result reuses the six-double pose record four times. It is
zeroed on entry and published only after every candidate is finite and a
proper rotation. Ada rejects every correspondence count other than three
before borrowing Core Mats. Pixel observations, K, and five distortion
coefficients are passed through; native `solveP3P` undistorts. No candidate
is selected. See [AP3P contract](ap3p-contract.md).

## Task 011 visibility boundary

One additional guarded C operation brings the private ABI to 45 operations.
Existing candidate records are reused; the new caller-owned count/four-byte
result is cleared and atomically published. Ada validates all hypotheses and
observations, handles pure/near rotation and no-reference dispositions without
native entry, and compacts selected correspondence positions into Core-owned
Float64 C2 Mats. Borrowed callbacks permit private continuous Float32 snapshots.
Native filtering uses no mask. Public output preserves all candidate positions.
See [visibility contract](planar-visibility-contract.md).

## Task 010 fixed-value decomposition

Calibrated homography decomposition adds one guarded private C call (44 total
imports/exports), with caller-owned storage for count and four fifteen-double
candidates. Output is zeroed before validation and published only after every
candidate passes. Public hypotheses are Ada values, not handles. Core ownership
and callback-scoped Mat borrowing remain unchanged.
See [planar contract](planar-motion-contract.md).

Task 008 adds a pure-Ada bearing/quality layer above normalized observations,
relative pose and public triangulation values. It shares Task 007 SO(3)
validation and scaled normalization, uses R^T to compare first-frame bearings,
and never enters the native ABI. Core ownership, 43 C imports/exports, and native
triangulation classification are unchanged. See
[quality contract](triangulation-quality-contract.md).

Normalized triangulation reuses the relative-pose ABI record and Core callback
boundary, snapshots observations, and returns only discriminated Ada values.
The private owned batch vector is published atomically. See
[triangulation contract](triangulation-contract.md) for gauge and status policy.

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


Task 005 adds distinct fundamental/options C records, six opaque-result exports
and one shared options-profile validation export. The fixed common legacy RANSAC
call uses no maximum-iteration argument (native ceiling 1000) and requires >=15
in both layers to avoid hidden LMeDS fallback. Native estimation rounds to
Float32; final-F classification uses original Float64 observations, maximum of
two point-to-line errors and no upstream mask. Pure-Ada public diagnostics use
scaled hypot and undefined geometry rather than epsilon/clamping. At least seven
final-model inliers are required. Ada independently calculates its final public
classification. Ownership/exception and macOS fault isolation remain unchanged.

## Task 009 numerical kernel

Point refinement is an implementation-only pure-Ada child package, separate from
the native C ABI. The public wrapper reuses relative-pose validation and parallax
measurement; the kernel implements analytic reprojection derivatives, maximum-entry
Jacobian scaling and checked 3x3 Cholesky with bounded damping. Only point XYZ is
optimized, at fixed first-camera/unit-baseline geometry. No native imports or
exports were added (43 unchanged). See [contract](triangulation-refinement-contract.md).
