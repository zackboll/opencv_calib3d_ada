# OpenCV Calib3D for Ada

Handwritten thick Ada binding for the narrow camera-pose portion of OpenCV needed
for 3-D-to-2-D pose estimation. Repository: `opencv_calib3d_ada`; Alire crate:
`opencv_calib3d`; public package: `OpenCV.Calib3D`.

**Version 0.1.0-dev. This ZIP is a bootstrap, not a qualified release.**
The source, tests, CI topology, raw-boundary harness and sanitizer driver are
included, but this creation environment does not contain GNAT/Alire/OpenCV, so no
native Ada build or AUnit pass is claimed here. Start with
[`docs/tasks/001-validate-bootstrap.md`](docs/tasks/001-validate-bootstrap.md).

## Native backend split

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

The frame convention is part of the API contract:

```text
X_camera = R * X_world + t
C_world  = -R^T * t
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
```

On Debian/Ubuntu:

```sh
sudo apt-get update
sudo apt-get install -y build-essential pkg-config libopencv-dev
```

The bootstrap depends on `opencv_core ~0.4.0` and pins Core to current reviewed
main commit `4da9d35ea21e1b2efe96296243ea668b488c6326` in the root, tests and
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
outliers, runs RANSAC PnP and prints pose/inlier/camera-center data. It has no
image files, GUI, camera, Features, DTED or geospatial dependency.

Synthetic thresholds and errors are demonstration fixtures, not navigation
accuracy claims.

## CI policy

PR CI is Linux, macOS, repository checks and Linux ASan/UBSan. Windows/MSYS2 is
in a separate **push-to-main-only** workflow because it is intentionally too slow
for the review loop. A manual matrix source-builds OpenCV 4.1.0, 4.10.0 and 5.0.0.
The presence of those workflows is not evidence that this bootstrap passed them.

## Deliberate exclusions

This first slice does not bind camera calibration, chessboards, stereo,
homography, essential/fundamental matrices, triangulation, USAC configuration,
PnP refinement, P3P/AP3P/IPPE/SQPNP selection, fisheye, undistortion, DTED,
Features, optical flow, or estimator/fusion policy.

The caller defines the world frame. A later navigation layer may use a local
metric terrain frame derived from DTED/geospatial data, but this crate does not
assign geographic meaning to `(X,Y,Z)`.

## Repository initialization

The ZIP contains source files only; it has no `.git` directory or credentials.
After reviewing it:

```sh
git init -b main
git add .
git commit -m "Bootstrap OpenCV Calib3D Ada binding"
```

## License

Apache-2.0; see `LICENSE` and `NOTICE`.
