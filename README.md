# OpenCV Calib3D for Ada

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
and [executed qualification](docs/fundamental-validation.md). Essential/relative
pose is reserved for Task 006.

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
iteratively and prints before/after RMS/max, pose and camera-center data. It has no
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

## CI policy

PR CI is Linux, macOS, repository checks and Linux ASan/UBSan. Windows/MSYS2 is
in a separate **push-to-main-only** workflow because it is intentionally too slow
for the review loop. A manual matrix source-builds OpenCV 4.1.0, 4.10.0 and 5.0.0.
The presence of those workflows alone is not evidence that they passed.

## Deliberate exclusions

This first slice does not bind camera calibration, chessboards, stereo,
essential matrices, triangulation, affine estimators, image warping, USAC configuration,
`solvePnPRefineLM`/VVS, P3P/AP3P/IPPE/SQPNP selection, fisheye, image undistortion/remap, DTED,
Features, optical flow, or estimator/fusion policy.

The caller defines the world frame. A later navigation layer may use a local
metric terrain frame derived from DTED/geospatial data, but this crate does not
assign geographic meaning to `(X,Y,Z)`.

## License

Apache-2.0; see `LICENSE` and `NOTICE`.
