# Task 005 qualification

## Starting state and Task 004 Windows gate

Starting clean main: `209398bb114716a1e1fe8dcaebd59e75f53d96c7`.
Branch: `feature/005-robust-fundamental`. Version remains 0.1.0-dev;
Core pin remains `4da9d35ea21e1b2efe96296243ea668b488c6326` in all roots.

Task 004 Windows post-merge run
<https://github.com/zackboll/opencv_calib3d_ada/actions/runs/37555089336>
finished **PASS** at the starting main SHA; final result retrieved once, complete
log inspected once. OpenCV 5.0.0 / geometry; MSYS2 MinGW64 GCC 16.2.0-4 native
compiler (not GNAT's bundled g++). AUnit: 40 registered, 40 executed, 40 passed,
zero failed assertions, zero unexpected errors. Homography cases actually ran:
known mapping/scale/infinity/range, clean 24/24 with maximum mapping error
2.21942581727039e-6, outlier support 21 with 2/7/15 rejected and every final-model
inclusion/exclusion checked, real collinear successful no-model, four rejected /
five robust path, invalid points/options, C/Ada layout/interchange.
DLL and DLL import library present, static shim archive absent; all configured
OpenCV geometry/Core and Core-shim import-library paths existed. PE inspection
confirmed `libopencv_geometry-500.dll` and `libopencv_core_shim.dll` imports.
This is Task 004 evidence, **not Task 005 Windows qualification**. Windows PR CI
was not added.

## Local executed evidence

Linux, OpenCV 4.10.0 / calib3d, Alire 2.1.1, GNAT 16.1.0, GPRbuild 26.0.1,
native g++ Debian 14.2.0-19. Warnings remain errors.
Production/tests builds and explicit AUnit run passed: **48 registered, 48
executed, 48 passed; 0 failed assertions, 0 unexpected errors**.

* Independent horizontal known-F oracle: 0 pixels for equal y, 3 pixels for
  y2-y1=3; F, -7F, 1e6F agree within Float64 tolerance; zero first/second normals
  undefined. Large/tiny scales, overflow and all-field nonfinite inputs tested.
* Direct pinhole fixture: 32 noncoplanar positive-depth 3-D points, fx=800,
  fy=820, cx=320, cy=240, horizontal baseline 0.75. No projection-binding oracle.
  Clean final support 32; maximum final-model epipolar error
  **5.12380385263952e-11 pixels** locally. No coefficient oracle or scale/sign
  assumption. Portable assertion permits near-total support rather than exact 32.
* Gross vertical outliers 2/7/15: final support **29**, all three rejected.
  Every included and excluded pair checked against final-F Float64 metric;
  indices valid, unique, strictly ascending and one-based positions even with
  non-one caller array bounds. Portable outlier support requirement is >=7.
* 0..14 rejected; 15 reaches true native RANSAC. Raw and public Ada armed stage-23
  bad_alloc checkpoint survives 14 rejection, is consumed by 15, and a following
  valid call succeeds.
* Real collinear fixture: native status success, Found=False, zero support,
  empty 1..0 array, inaccessible matrix. No forced-no-model control needed locally.
* Fundamental C/Ada layout: C sizeof=72, alignment=8, nine offsets
  0,8,16,24,32,40,48,56,64 equal Ada Size/Alignment/Position; C writes all nine
  distinct values read by Ada. Options: sizeof=16, alignment=8, offsets 0,8;
  complete compiler-derived two-field layout and C-written interchange.
* Raw production and raw fault actual-shim driver passed: real strided Regions
  for both inputs, nulls, 0..14/count mismatch, Float32 C2/Float64 C1/C3/wrong
  geometry/N-D, both sets nonfinite, invalid options including squared-float
  underflow/overflow and native confidence boundaries, no-model/result access,
  output clearing and null destruction. No fake opaque pointers.
* Fundamental stages 23..28 times five exception classes: **30 raw** and
  **30 Ada** fault scenarios passed. Existing qualified macOS libc++ isolation
  is reused. Unknown exceptions, std exception, bad_alloc, cv::Exception and
  invalid_argument cannot cross the C ABI; no partial result publication.
* Actual shim production/fault **ASan+UBSan PASS**, leak detection enabled,
  halt-on-error, no suppressions; production binary has no test-control symbols.
  Linked OpenCV/Core are **not** claimed fully sanitizer-instrumented.
* `alr test` passed (full AUnit/raw/fault campaign).
* Repository checker passed: 27 private C exports/imports, 48 registrations,
  identical Core pins/no bridge vendoring/private imports/backend/CI topology.
* Python/static tests: 22 executed/passed at final local qualification.
* Four standalone C/C++ helpers pass (header, PnP profile, homography profile,
  fundamental profile), plus compiler-derived C/Ada layout/interchange helper.
* Seven shell scripts pass `sh -n`; `git diff --check` passes.
* All three file-free deterministic examples build and run: pnp_synthetic,
  homography_synthetic, fundamental_synthetic. Fundamental example prints 32
  pairs, Found=True, support 29, 2/7/15 did not survive, maximum accepted error
  5.12380385263952e-11, sample error 7.78930351570759e-14.

## Remote gates

Ordinary PR CI and the once-only manual pinned compatibility matrix are pending.
No remote pass is inferred from local tests or workflow configuration. Record
actual run links, head SHA and each target's evidence here after execution.

## Limitations

Native estimation rounds point observations to Float32. Fixed 1000-iteration
ceiling, no public robust-method/iteration policy. No determinant/rank threshold,
coefficient equality or exact cross-version support promise. Finite geometry may
be unrepresentable and yield undefined error. Synthetic tests/fault controls do
not demonstrate real allocator exhaustion, navigation accuracy, or correct
physical correspondence. No Essential matrix, recovered pose, triangulation,
metric scale/position/terrain recovery; those are not implemented in this task.
