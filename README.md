# OpenCV Calib3D for Ada

## Task 023: epipolar-gated three-view metric landmarks

`Qualify_Three_View_Landmarks` screens first–second correspondences using known-pose
normalized Sampson errors, reconstructs metric world points with optional fixed-pose
refinement, selects a fitting-stage-approved initial/refined candidate, and only
then verifies it against the third camera. Three calibrations and known poses are
independent inputs; third pixels never enter fitting or candidate selection, and
third rejection never triggers a fallback retry. All limits are caller-selected,
finite, nonnegative and inclusive. Per-track results retain original positions,
screening, initial/refinement diagnostics and guarded candidate/accepted geometry.
`Three_View_Landmark_Observations` compacts only accepted coordinates into the
existing owned landmark set, optionally paired with fourth-image observations.

The third image adds geometric evidence, not true statistical independence,
correct feature identity, global localization, physical ground truth or an
uncertainty bound. No C++/ABI change (48 operations), new solver or dependency.
See [contract](docs/three-view-landmark-contract.md) and
[validation](docs/three-view-landmark-validation.md). The eleventh synthetic example
includes epipolar mismatches, epipolar-consistent wrong depth and third mismatches:

```sh
alr -n -C examples build
alr -n -C examples exec -- sh ../scripts/run_native.sh bin/three_view_landmarks_synthetic
```

## Task 022: known-pose calibrated two-view epipolar verification

`Essential_From_Camera_Poses` builds `E=[normalize(t_relative)]_x R_relative`
from two supplied world-to-camera poses. The relative translation term is in the
**second camera frame**, not world camera-center displacement. Physical baseline
is retained separately; exact zero baseline (including pure rotation) publishes
no Essential matrix. No pose, E, or calibration is estimated or changed.

`Verify_Known_Pose_Epipolar` separately undistorts each camera's pixels, evaluates
the existing normalized Sampson metric for every pair, and applies mandatory
caller-selected inclusive RMS and per-point/maximum limits. These errors are
dimensionless, **not pixels**. Owned diagnostics guard absent E and whole-set
scores; undefined errors never become zero or a partial-set RMS. An overload
consumes two accepted Task 021 relocalizations without rerunning fitting.
Cross-image match independence remains the caller's responsibility.

This verifies geometric consistency, not absolute position, physical baseline
magnitude, E sign, positive depth, reconstruction accuracy, matching correctness
or localization uncertainty. Production C++ and the 48-operation ABI are unchanged.
See the [contract](docs/known-pose-epipolar-contract.md) and
[validation](docs/known-pose-epipolar-validation.md).

```sh
alr -n -C examples build
alr -n -C examples exec -- sh ../scripts/run_native.sh bin/known_pose_epipolar_synthetic
```

The tenth file-free synthetic example accepts independently projected clean
matches, rejects corrupted matches, and demonstrates zero-baseline unavailability.
It is not physical sensor or navigation evidence.

## Task 021: independently verified landmark camera relocalization

`Relocalize_Camera` robustly fits a new world-to-camera pose from at least six
trusted Cartesian landmark/pixel pairs using existing fixed-EPNP RANSAC, copies
the actual native consensus positions, and refines **only** those pairs once.
Separate disjoint held-out landmarks verify both poses. Explicit caller limits
govern final consensus RMS, held-out RMS, maximum error, and RMS worsening.
Native optimizer success alone is never acceptance. The owned private result
guards initial/accepted poses, camera center and available whole-set diagnostics.
`Accepted_Landmark_Observations` optionally compacts accepted Task 020 values and
new-image pixels, preserving original one-based positions. Neither landmark
acceptance nor held-out compatibility establishes independent physical truth,
global localization correctness, statistical independence or uncertainty bounds.
No C++/ABI change (48 operations), new solver, or navigation dependency.
See [contract](docs/landmark-relocalization-contract.md) and
[validation](docs/landmark-relocalization-validation.md).

```sh
alr -n -C examples build
alr -n -C examples exec -- sh ../scripts/run_native.sh bin/landmark_relocalization_synthetic
```

This ninth file-free example uses independent scalar analytic projections,
gross fitting mismatches and separate held-out landmarks. It reports actual
consensus, before/after held-out and final inlier scores, physical rotation and
translation differences, and camera-center error. These are synthetic observations,
not sensor data or evidence of real navigation accuracy.

## Task 020: fixed-pose metric world-point refinement

`Refine_Metric_World_Points` reconstructs once, refines eligible unit-baseline
first-camera points with the existing pure-Ada solver, scales by the known
baseline, and checks the representable world point in both calibrated cameras.
Both poses, baseline, intrinsics and distortion remain fixed. Every original
metric result is retained, including initial policy rejections that refinement
may rescue. A private owned result separates optimizer improvement, candidate
diagnostics, final acceptance, and guarded accepted world coordinates. All four
quality thresholds and both per-camera pixel-worsening allowances are selected
by the caller, inclusive without epsilon. These fitting diagnostics do not prove
world accuracy or statistical independence. No covariance, depth uncertainty,
bundle adjustment, pose optimization, DTED/geographic conversion or navigation
fusion is included. C++ is unchanged; private ABI remains 48.
See the [contract](docs/metric-point-refinement-contract.md) and
[qualification](docs/metric-point-refinement-validation.md).

```sh
alr -n -C examples build
alr -n -C examples exec -- sh ../scripts/run_native.sh bin/metric_refinement_synthetic
```

This eighth example uses synthetic observations, not actual sensor data.

## Task 019: calibrated metric world-frame triangulation

`Triangulate_World_Points` accepts independently calibrated distorted pixels,
two known poses in one Cartesian world frame, and four explicit caller quality
thresholds. It reuses native normalization and unit-baseline triangulation,
scales by the known-pose physical baseline, transforms to world coordinates, and
checks both native distorted pixel projections. Source status is preserved;
optional coordinates, depths, and residuals are discriminant-guarded. Exact zero
baseline never fabricates a point. No solver or C ABI operation is added (48).
See the [contract](docs/metric-triangulation-contract.md) and
[validation](docs/metric-triangulation-validation.md).

Run the file-free independent synthetic oracle:

```sh
alr -n -C examples build
alr -n -C examples exec -- sh ../scripts/run_native.sh bin/metric_triangulation_synthetic
```


## Task 017: reference-validated AP3P pose refinement

`Select_Refine_AP3P_Pose` solves three seed correspondences, applies the
existing Task 016 selection policy, and refines the owned pose with the
existing iterative `solvePnP` binding. Selection references may enter that
refinement. Held-out references are used only to accept or reject it.
Acceptance thresholds have no defaults and are not fitting residuals.
The published pose remains `X_camera = R * X_world + t` in the caller's
Cartesian frame, not an Earth or DTED frame. No new C ABI operation is added.
See the [contract](docs/ap3p-refinement-contract.md) and
[qualification](docs/ap3p-refinement-validation.md).

## Task 016: reference-qualified AP3P pose selection

`Select_AP3P_Hypothesis` applies a caller-supplied RMS, maximum-error, and
strict separation policy to AP3P hypotheses already scored by
`Evaluate_AP3P_References`. `Select_AP3P_Pose` solves the existing
three-point AP3P problem, evaluates additional independent world points, and
returns a private result that owns the selected pose from that solve. Thresholds have no defaults. A unique
pose is conditional on those references and that policy; it is not global
localization. Translation remains `t` in the caller's Cartesian frame. No new
C ABI operation is added. See the [contract](docs/ap3p-selection-contract.md)
and [qualification](docs/ap3p-selection-validation.md).

## Task 015: guarded native epipolar correspondence correction

`Correct_Epipolar_Matches` binds native OpenCV `cv::correctMatches` under a
guarded, all-or-error contract. OpenCV performs the Hartley–Sturm correction;
Ada and the shim validate inputs, normalize F, apply **caller-selected** SVD
conditioning and input-line-quality policies (no defaults), and verify finite
output, the corrected epipolar residual and a one-sided displacement bound. That
bound is a **necessary** optimality condition only: success is not a
global-optimality certificate, and native output may be rejected even when OpenCV
returns normally. No pose, metric scale or position is recovered. See the
[contract](docs/epipolar-correction-contract.md), [source review and research
findings](docs/epipolar-correction-source-review.md) and
[qualification](docs/epipolar-correction-validation.md).

## Task 014: native pose composition

`Compose_Poses (First, Second)` chains two world-to-camera rigid transforms
through native `cv::composeRT` (apply `First`, then `Second`). Rotation vectors
are returned as native Rodrigues output; compare `Rotation_Matrix_Of`, not
components. Inputs must be consecutive frames in consistent Cartesian units.
This is coordinate-frame composition, not navigation fusion or geolocation.
See the [contract](docs/pose-composition-contract.md) and
[qualification](docs/pose-composition-validation.md).

## Task 018: camera pose inversion and relative motion

`Invert_Pose` inverts a rigid source-to-destination transform stored in `World_To_Camera_Pose`. `Camera_To_Camera_Pose (First, Second)` maps first-camera coordinates to second-camera coordinates by composing the inverse of `First` with `Second`. `Measure_Camera_Motion` reports both world camera centers, the world displacement `C2 - C1`, and its scaled baseline length. Relative translation is in the second camera frame and is not that displacement. `t` is not a camera position. Scale exists only because both absolute poses use the same caller Cartesian frame. No estimator, geographic frame, or navigation uncertainty is implied. See the [contract](docs/relative-camera-motion-contract.md) and [qualification](docs/relative-camera-motion-validation.md).

## Task 013: AP3P independent-reference diagnostics

`Evaluate_AP3P_References` classifies additional world/pixel observations
against every retained AP3P hypothesis. Native `Project_Points` performs
perspective projection and five-coefficient Brown distortion. Ada computes
camera depth `Z` from `X_camera = R * X_world + t` and aggregates RMS and
maximum error with `Summarize_Reprojection`. A native projection failure
raises `OpenCV_Error`; it is not reported as `Numerically_Unavailable`.
No hypothesis is selected or refined. See
[contract](docs/ap3p-reference-contract.md) and
[qualification](docs/ap3p-reference-validation.md).

## Task 012: minimal AP3P pose hypotheses

`Solve_AP3P_Hypotheses` returns every native `SOLVEPNP_AP3P` hypothesis for
exactly three world points and three pixel observations. It does not select
a pose. OpenCV 4.1 requires three points; later versions also allow four,
but this API does not. Zero native solutions is an empty success. Translation
uses world-point units and is not the camera position. See
[contract](docs/ap3p-contract.md), [source review](docs/ap3p-source-review.md),
and [qualification](docs/ap3p-validation.md).

## Task 011: calibrated homography visibility

`Assess_Planar_Visibility` returns a disposition for every original motion
hypothesis. Omitted selection tests all references; explicitly empty selection
tests none. Pure/near rotation is not applicable, and zero survivors is valid.
Strict signs are `n dot p1 > 0` and `(R*n) dot p2 > 0`, evaluated using checked
native Float32 observations. Passing is not full cheirality or unique camera
motion selection; multiple candidates may remain. No metric scale is recovered.
See [contract](docs/planar-visibility-contract.md),
[immutable source review](docs/planar-visibility-source-review.md), and
[qualification](docs/planar-visibility-validation.md).

## Task 010: calibrated planar motion

`OpenCV.Calib3D.Decompose_Calibrated_Homography` returns all native motion
hypotheses as owned Ada values, including pure rotation. It requires an
undistorted pinhole homography and shared intrinsics; identity K supports
normalized-coordinate homographies. Translation is relative to unknown plane
distance, not metric translation. No physical hypothesis is selected.
See [contract](docs/planar-motion-contract.md),
[source review](docs/planar-motion-source-review.md), and
[validation](docs/planar-motion-validation.md). The existing
`homography_synthetic` example demonstrates both pure and general motion.

Pure-Ada `Measure_Stereo_Parallax` and `Assess_Triangulation` expose directed
ray angles (0..pi), acute line angles (0..pi/2), and caller-selected inclusive
parallax/residual quality flags. Numerically `Usable` does not mean geometrically
well-conditioned. Defaults preserve validity; no universal cutoff or uncertainty
model is implied. See [quality contract](docs/triangulation-quality-contract.md)
and [qualification](docs/triangulation-quality-validation.md). The existing
`triangulation_synthetic` example demonstrates usable, weak-angle and non-Usable
correspondences; the API uses radians and the example also prints degrees.

Normalized two-view reconstruction is available through `Triangulate_Normalized`.
Positions are **first-camera unit-baseline coordinates, not meters**; no metric
baseline is recovered. See [contract](docs/triangulation-contract.md),
[source review](docs/triangulation-source-review.md), and the explicitly
[qualification report](docs/triangulation-validation.md).
The fifth file-free example is `triangulation_synthetic`.

Handwritten thick Ada binding for camera pose and robust planar/projective
and two-view epipolar 2-D correspondence verification. Repository: `opencv_calib3d_ada`; Alire crate:
`opencv_calib3d`; public package: `OpenCV.Calib3D`.

**Version 0.1.0-dev; review baseline, not a release.**
The generated bootstrap originally had static checks only. Task 001 has now run
local native qualification on OpenCV **4.10.0 / calib3d**: production/tests/example
builds, 12/12 AUnit cases, raw ABI, C/Ada layouts and both actual-shim ASan/UBSan
variants pass. Cross-platform and pinned-matrix evidence is recorded separately
in [`docs/bootstrap-validation.md`](docs/bootstrap-validation.md) and the PR.

Task 002 adds iterative initial-guess refinement and Ada reprojection diagnostics.
Local OpenCV **4.10.0 / calib3d** qualification passes **19/19 AUnit**, raw ABI,
public false/exception atomicity, production/fault ASan+UBSan, and the extended
example. Remote review gates are tracked separately; no new release is claimed.

Task 005 adds `Fundamental_Matrix`, pure-Ada `Maximum_Epipolar_Error`, and owned
`Fundamental_Estimate` from `Estimate_Fundamental_RANSAC`. Equal counts **>=15**
prevent the native 8..14 LMeDS fallback. The common native iteration ceiling is
fixed at **1000**, with no iteration option. Native estimation rounds observations
to Float32; public final-F inliers use the original Float64 points and the maximum
of both point-to-epipolar-line distances (not Sampson distance). At least seven
final inliers are required. Threshold squared-Float32 representability and the
inclusive native DBL_EPSILON confidence interval prevent silent policy changes.
F alone does not recover metric scale, camera position, or 3-D terrain location.
See [contract](docs/fundamental-contract.md), [immutable source review](docs/fundamental-source-review.md),
and [executed qualification](docs/fundamental-validation.md).

Task 006 adds normalized-only `Essential_Matrix`, `Normalized_Sampson_Error`,
`Essential_Estimate` from `Estimate_Essential_RANSAC`, automatic relative pose
recovery, separate Essential/pose inlier lists, and
`Second_Camera_Center_Direction_In_First`. **Do NOT pass raw distorted pixels:**
use `Undistort_To_Normalized` separately for each camera first. Equal counts
**>=6** ensure actual five-point subset consensus; fixed native ceiling **1000**.
Observations/five-point algebra are Float64, native error-mask comparison is
Float32; public final-E inliers use original Float64 normalized Sampson error.
Both Essential and cheirality-qualified pose require five supporting pairs.
Recovery uses exactly final-E support and an explicit effectively-unbounded
finite depth threshold, not the native default 50 baseline units.
**Metric baseline magnitude is unknown. Translation_Direction is not camera
position.** The relative convention is `X_second = R * X_first + lambda*t_hat`;
camera-center motion direction in first coordinates is `normalize(-R^T*t_hat)`.
See [contract](docs/essential-contract.md),
[immutable source review](docs/essential-source-review.md), and
[qualification evidence](docs/essential-validation.md).

## Native backend split

Task 004 adds `Homography_Matrix`, safe pure-Ada `Map_With_Homography`, and owned
`Homography_Estimate` from fixed legacy `Estimate_Homography_RANSAC`.
**Equal counts >=5** avoid OpenCV's four-point RANSAC bypass. Public inliers are
independently reclassified against **final H with original Float64 points**, not
the upstream mask (whose refinement semantics differ between 4.x and 5.0).
The threshold is forward Euclidean error in destination-image pixels.

**A single homography models planes/projective image registration, not general
3-D terrain pose.** Use PnP for actual 3-D world points. See the
[homography contract](docs/homography-contract.md),
[immutable source review](docs/homography-source-review.md), and
[executed validation](docs/homography-validation.md).

Task 003 adds the geometric bridge:

```text
distorted pixel -> normalized camera coordinate -> camera ray -> world ray
```

`Undistort_To_Normalized`, `Camera_Bearing_Rays`, `World_Bearing_Rays`,
`Rotation_Matrix_Of`, and explicit world/camera point and direction transforms
return Ada values. Normalized coordinates/directions are dimensionless; ray origins
use the caller's Cartesian world units. Native standard undistortion uses fixed
COUNT|EPS, 20 iterations, 1e-12 pixel epsilon, with no R/P or public tuning knob.
See [the camera-ray contract](docs/camera-rays-contract.md) for equations, units,
validation/empty semantics and limitations; actual qualification is recorded in
[`docs/bootstrap-validation.md`](docs/bootstrap-validation.md).

The Ada API is intentionally stable while OpenCV moved the native implementation:

```text
OpenCV 4.1 .. 4.x  -> opencv2/calib3d.hpp      -> opencv_calib3d
OpenCV 5.0         -> opencv2/geometry/3d.hpp  -> opencv_geometry
Ada                 -> OpenCV.Calib3D on both
```

This crate does **not** use the Ada `OpenCV.Geometry` namespace; that namespace
belongs to the separate geometry binding.

## Initial API

The bootstrap exposes:

- `Camera_Intrinsics` with `fx`, `fy`, `cx`, `cy` and zero skew;
- five Brown/OpenCV distortion coefficients `k1,k2,p1,p2,k3`;
- `Project_Points`;
- `World_To_Camera_Pose` using an OpenCV Rodrigues rotation vector;
- `Camera_Center`;
- fixed-flag `Solve_PnP_RANSAC` using `SOLVEPNP_EPNP`;
- owned, one-based RANSAC inlier indices;
- explicit success/no-pose result state.
- `Reprojection_Errors` and `Summarize_Reprojection`: Euclidean errors, RMS and
  maximum in pixels, with robust scaled arithmetic and valid empty summaries;
- `Refine_Pose_Iterative`: `solvePnP` with a supplied extrinsic guess and fixed
  `SOLVEPNP_ITERATIVE`, common to OpenCV 4.1/4.10/5.0, **not `solvePnPRefineLM`**.

Refinement requires equal counts >=4, a finite initial pose and finite validated
points/camera/distortion. Four is a conservative Ada contract; native OpenCV also
permits three with a guess. Native false keeps the initial pose exactly unchanged;
errors never publish partial pose data. Diagnostics do not imply pose uncertainty
or covariance; low pixel error does not prove globally correct localization.

The frame convention is part of the API contract:

```text
X_camera = R * X_world + t
C_world  = -R^T * t
X_world(s) = C_world + s * d_world, s > 0
```

`Translation` is `t`; **it is not the camera position**.

## Build

With Alire, an Ada 2022 toolchain, C++17, pkg-config and OpenCV development
packages installed:

```sh
alr -n build
alr test
alr -n -C examples build
alr -n -C examples exec -- sh ../scripts/run_native.sh bin/pnp_synthetic
alr -n -C examples exec -- sh ../scripts/run_native.sh bin/homography_synthetic
alr -n -C examples exec -- sh ../scripts/run_native.sh bin/fundamental_synthetic
alr -n -C examples exec -- sh ../scripts/run_native.sh bin/essential_synthetic
```

On Debian/Ubuntu:

```sh
sudo apt-get update
sudo apt-get install -y build-essential pkg-config libopencv-dev
```

The binding depends on `opencv_core ~0.4.0` and retains bootstrap Core
commit `4da9d35ea21e1b2efe96296243ea668b488c6326` in the root, tests and
examples Alire roots. Core is fetched; it is not bundled or copied.

## Ownership architecture

```text
Ada application
    OpenCV.Calib3D
        private Ada C ABI
            fixed-width C records + opaque temporary result
                C++17 shim
                    OpenCV 4: calib3d
                    OpenCV 5: geometry

Application Mat ownership: OpenCV.Core
Native Mat borrowing: OpenCV.Core.Module_Interop callback scope only
```

Public 2-D/3-D values reuse `OpenCV.Core.Float64_Vec2/Vec3.Vector`. Point arrays
are converted into Core-owned `Float64 C2/C3` Mats before entering native OpenCV.
No STL vector, `cv::Mat`, native pointer, or native result handle appears in the
public API.

## Synthetic pose example

`examples/src/pnp_synthetic.adb` creates a noncoplanar 3-D point set, projects it
from a known pose, corrupts three 2-D correspondences with large synthetic
outliers, runs RANSAC PnP, builds local accepted-correspondence arrays, refines
iteratively and prints before/after RMS/max, pose and camera-center data.
It also selects an AP3P pose, refines it, and prints held-out RMS before and
after that refinement. Fitting error and held-out error are different quantities.
It has no
image files, GUI, camera, Features, DTED or geospatial dependency.
It then prints principal/off-axis pixels, normalized coordinates, camera unit
directions, world ray origins and world unit directions. These are metric-frame
geometric rays only; no DTED/geodetic interpretation or intersection is performed.

Synthetic thresholds and errors are demonstration fixtures, not navigation
accuracy claims.

The separate file-free `examples/src/homography_synthetic.adb` demonstrates
planar/projective verification: 24 generated correspondences, three gross
outliers, final-model support, scale-ambiguous H, and sample mapping error. It
does not estimate camera pose or terrain localization and adds no dependencies.

The fourth file-free `essential_synthetic` example directly constructs normalized
two-view observations from noncoplanar 3-D points, nonidentity R and translation,
without a projection binding. It injects outliers 2/7/15 and prints final E,
relative rotation, signed unit translation-term direction, second-camera-center
direction, support and geometric diagnostics. No files, GUI or hardware. This
is calibrated two-view relative geometry, **not metric navigation position**.

## CI policy

PR CI is Linux, macOS, repository checks and Linux ASan/UBSan. Windows/MSYS2
stays in a separate workflow and still runs on every push to `main`. Pull
requests run that Windows job only when `.github/workflows/windows-post-merge.yml`
itself changes, so unrelated feature PRs do not pay for it. A manual matrix
source-builds OpenCV 4.1.0, 4.10.0 and 5.0.0.
The presence of those workflows alone is not evidence that they passed.

## Deliberate exclusions

This first slice does not bind camera calibration, chessboards, stereo,
public triangulation/decomposition, affine estimators, image warping, USAC configuration,
`solvePnPRefineLM`/VVS, IPPE/SQPNP, fisheye, image undistortion/remap, DTED,
Features, optical flow, or estimator/fusion policy.

The caller defines the world frame. A later navigation layer may use a local
metric terrain frame derived from DTED/geospatial data, but this crate does not
assign geographic meaning to `(X,Y,Z)`.

## License

Apache-2.0; see `LICENSE` and `NOTICE`.

## Point-only normalized refinement

`Refine_Triangulated_Points` adds bounded pure-Ada damped Gauss-Newton after
linear triangulation and caller parallax screening. Pose and unit-baseline gauge
stay fixed; only strictly decreasing, positive-depth updates publish new point
coordinates and recomputed diagnostics. Inputs remain immutable. Default budget
is 20 (maximum 100), and default minimum parallax is zero.
See [contract](docs/triangulation-refinement-contract.md) and
[executed qualification](docs/triangulation-refinement-validation.md).
`triangulation_synthetic` demonstrates clean/noisy correspondences. This minimizes
normalized reprojection residual, not metric 3-D error or covariance.
