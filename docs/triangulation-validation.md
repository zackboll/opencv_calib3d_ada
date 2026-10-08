# Task 007 triangulation qualification

Starting main: `5a9fcfb50ea3dc1332309832b4e7c9c2fa55832a`.
Branch: `feature/007-normalized-triangulation`.
Core pin: `4da9d35ea21e1b2efe96296243ea668b488c6326`; version `0.1.0-dev`.

## Task 006 Windows

[Run 37708768020](https://github.com/zackboll/opencv_calib3d_ada/actions/runs/37708768020)
PASS at starting main. Logs qualify OpenCV 5.0.0/geometry, MSYS2 MinGW64 g++,
DLL/import libraries, 57 registered/executed/passed AUnit tests, zero failed
assertions/unexpected errors, Essential/relative-pose execution, and PE imports
`libopencv_geometry-500.dll` and `libopencv_core_shim.dll`. This is Task 006
evidence, not Task 007 triangulation evidence. Windows stays post-merge-only.

## Local OpenCV 4.10.0/calib3d

* Production and tests builds PASS; native AUnit 64 registered/executed/passed,
  zero failed assertions/unexpected errors; `alr test` PASS.
* Rectified independent (2,1,5) disparity oracle and both depths 5 PASS at 1e-9;
  errors <1e-12. Negative disparity -> Non_Positive_Depth.
* Forty nonidentity baseline-scaled points PASS at 1e-8; inverse-frame PASS.
* Independently reprojected noisy diagnostics PASS.
* Essential composition: 40 selected, 40 usable, all unusable counts zero;
  maximum normalized error `1.14439169963056E-15`.
* One/empty/distinct lower bounds PASS; malformed poses rejected even when empty.
* Compiler C/Ada record size 64 bytes, alignment 8; all eight field positions
  and C-written full-record interchange PASS.
* 43 matching private exports/imports; 24 Python tests PASS; six C/C++ profile
  helpers PASS; shell syntax PASS.
* Actual-shim ASan+UBSan production/fault PASS with leak detection, halt-on-error,
  no suppressions. Linked OpenCV/Core are not claimed fully instrumented.
  Includes real strided Core Regions, typed empty, rectified/behind ABI,
  complete requested schema/pose negatives, bounds, and 25 checkpoint 37..41 fault cases.
  Production test-control symbol exclusion PASS.
* Profile helper PASS: negative W, zero W, nonfinite H, tiny nonzero W,
  behind camera, nonrepresentable reprojection.
* All five examples build and run successfully through the native wrapper.

## Completed continuation coverage

* Analytic axial cameras: R=I, t=(0,0,-1), X1=(0.2,0,0.5), X2=(0.2,0,-0.5),
  observations (0.4,0)/(-0.4,0) -> Non_Positive_Depth. Reverse observations and
  t=(0,0,+1) qualify first-only negative depth. Expected depths +/-0.5 are
  independent analytic values; invalid public variants intentionally expose no depths.
* Real native triangulation precedes first-column test-only homogeneous injection.
  W=0, W=1e-300 (finite usable enormous point), nonfinite X/Y/Z/W, behind point,
  and finite positive-depth (100,1,1e-307) with overflowing normalized projection
  exercise every status. Mixed batches retain their other results.
* Accessor unknown integer 867 returns C success but Ada's explicit mapping raises
  OpenCV_Error. Test-only live-handle counter equals zero after public rejection.
* 25 public Ada checkpoint 37..41 exception translations PASS, all five exception
  classes; shared macOS libc++ isolation retained. Every handle cleaned up.
* Raw production/fault and sanitizer production/fault now exercise nonidentity,
  inverse frames, noisy observations, parallel/near-infinite rays, axial depths,
  one/empty, strided Regions, and Essential pose-inlier composition.
* Parallel native rays on local 4.10 produce Usable with depth about 9.69183e17,
  not exact zero W. This is not forced into At_Infinity by an epsilon.
* Local maximum coordinate errors: nonidentity 7.10542735760100e-15;
  inverse 1.06581410364015e-14. Raw Euclidean errors: 8.31408e-15/1.07983e-14.
* Rectified absolute X/Y/Z errors: 2.22044604925031e-16,
  2.22044604925031e-16, 8.88178419700125e-16.

## Remote gates

Ordinary PR CI and the manual pinned matrix are pending; no remote PASS is
claimed until completed logs are inspected. Tolerances are provisional until
the pinned matrix executes. No metric baseline/uncertainty is recovered;
linked OpenCV/Core are not fully sanitizer-instrumented.