# Task 008 qualification evidence

Starting main: `b7c6298d39c99b64177b5f04e5bfa74dc15ae990`.
Branch: `feature/008-triangulation-quality`. Version remains `0.1.0-dev`;
Core pin remains `4da9d35ea21e1b2efe96296243ea668b488c6326` in all roots.

## Gate 0: Task 007 post-merge

[Windows run 37714428650](https://github.com/zackboll/opencv_calib3d_ada/actions/runs/37714428650)
PASS. Exact completed logs show OpenCV 5.0.0/geometry, actual compiler
`C:/Users/runneradmin/AppData/Local/alire/cache/msys64/mingw64/bin/g++.exe`,
Task 007 triangulation execution, 64 registered/executed/passed AUnit tests,
DLL plus import-library checks (no static shim archive), and PE imports
`libopencv_geometry-500.dll` and `libopencv_core_shim.dll`.
[Cross-platform main run 37714428572](https://github.com/zackboll/opencv_calib3d_ada/actions/runs/37714428572)
PASS: repository-checks, linux, macos, linux-sanitizers. Windows remains
post-merge-only; this baseline is not Task 008 Windows qualification.

## Local serial campaign: OpenCV 4.10.0/calib3d

Executed the requested static/profile/shell, production/test build, native
AUnit, `alr test`, actual-shim sanitizer, examples build and all five synthetic
example commands serially. All PASS, including `git diff --check`.

* 74 registered/executed/passed AUnit cases; zero failed assertions and
  unexpected errors. All 64 Task 007 cases retained; ten additions.
* Repository static check PASS: unchanged 43 matching C exports/imports.
  24 Python configuration tests PASS; six standalone C/C++ helpers PASS;
  all repository shell scripts pass syntax checks.
* Raw production/fault ABI and Ada exception campaign PASS through `alr test`.
  Production/fault ASan+UBSan actual-shim variants PASS with leak detection,
  halt-on-error and no suppressions. Pure-Ada diagnostics are **not** claimed
  ASan-instrumented; linked OpenCV/Core are not fully instrumented.
* All five examples PASS: PnP, homography, fundamental, Essential, triangulation.

## Independent angular and policy results

* Rectified `atan(0.4)-atan(0.2)` = `0.183110817262484` rad; measured absolute
  error `0` locally, required tolerance 1e-15. Both angular definitions agree.
* Tiny angle `atan(1e-12)` measured `1e-12` rad, positive (tolerance 1e-27).
  Exact parallel measured zero for both angles.
* Exact Rz quarter-turn transpose oracle PASS within 1e-15; Y half-turn
  antiparallel oracle theta=pi, alpha=0 PASS within 1e-15, no triangulation needed.
* Task 006 nonidentity inverse-frame symmetry PASS at 1e-15; maximum directed
  angle difference `8.32667268468867e-17`. Positive translation scale 7 leaves
  both measurements exactly unchanged.
* Extreme finite coordinates at Float64'Last PASS without naive-square overflow.
* Inclusive below/equal/above angular and residual boundaries PASS; every
  usable combination of passing/failing limits and non-Usable result PASS.
  Non-Usable results retain parallax; usable input statuses remain unchanged.
* Native disparity `1e-8`: Usable, acute angle `1e-8` rad; caller minimum
  `1e-6` rejects solely on parallax, residual passes. No depth-magnitude oracle.
* Noisy Task 007 fixture: maximum residual `0.0150403822397045`; below rejects,
  equality/above accepts, native status remains Usable.
* Empty/one-point/distinct bounds, count mismatches, SO(3)/finite-pose errors,
  zero/nonfinite translation, nonfinite observations/options, and invalid
  manually constructed usable position/depth/error fields PASS.

## Essential composition

Forty clean selected correspondences, forty triangulated and forty assessments.
Finite in-range angles and preserved statuses PASS. Acute range
`0.0924445175692289 .. 0.184332877417093` rad. Usable/default accepted/nonzero
minimum(0.1)/residual maximum(1e-10) counts locally: **40 / 40 / 34 / 40**.
Nondefault counts are observations, not cross-version exact-count assertions.

## Remote review gates

Ordinary PR CI and pinned 4.1.0/calib3d, 4.10.0/calib3d, 5.0.0/geometry matrix
results will be recorded in the PR after inspecting completed logs. No remote
Task 008 PASS is claimed here before execution. Matrix dispatch occurs once
on stable final head after ordinary CI passes; WITH_ADE=OFF remains unchanged.

No C++ shim/header or private Ada C API changes, new exports, native handles,
fault hooks or dependencies. Existing numerical solver/status behavior is
unchanged. No covariance, accuracy guarantee, metric scale recovery, navigation
or geospatial claim. See [contract](triangulation-quality-contract.md).