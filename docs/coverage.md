# Coverage

Bootstrap public coverage:

- camera intrinsics (`fx`,`fy`,`cx`,`cy`, zero skew);
- five-coefficient Brown/OpenCV distortion model;
- world-to-camera Rodrigues pose value;
- camera-center conversion;
- sparse `projectPoints` without Jacobian;
- fixed-EPNP `solvePnPRansac` without extrinsic guess;
- explicit found/no-pose state;
- one-based sorted inlier indices.

The initial bootstrap registered 10 AUnit cases. Qualification registers 12:
identity/translation/rotation projection, all-five distortion, identity/rotated
camera center, clean/outlier PnP, native false/no-pose behavior, intrinsics,
options/count validation, and compiler-derived C/Ada layout/interchange for every
member of intrinsics/distortion/pose. Local OpenCV 4.10.0: 12 executed, 12 passed,
zero failed assertions and unexpected errors. The runner fails its process on
either kind of AUnit failure.

Python configuration/static suite: 16 tests, including isolated configure backend,
missing header/library, invalid version and CI topology rejection fixtures.
Raw ABI uses real Core-created matrices: null required arguments, depth/channel/
Nx1 schema failures, count mismatch/minimum, invalid camera/options/distortion,
inlier accessor range/null checks, cleared failure outputs and destroy(null).
It exercises robust outliers and native no-pose behavior in production and
fault-injection builds. Exception injection covers all five exception categories
before/after projection, before/after native solve and before publication, plus
camera-center and result-access stages. Projection failure preserves output;
scalar outputs clear and unpublished results are reclaimed.

Both actual-shim Linux ASan/UBSan variants passed locally with leak detection,
halt-on-error and no suppressions; production excludes test-hook symbols.
This does not prove real allocator exhaustion or instrument all dependencies.
See `bootstrap-validation.md` and PR Actions for cross-platform/matrix evidence.

Out of scope: broad calibration, stereo, homography, essential/fundamental,
triangulation, arbitrary SolvePnP methods, RefineLM/VVS, fisheye, image undistortion/remap,
USAC configuration, Features integration and geographic/DTED semantics.

## Task 002 inventory

The suite now registers **19 AUnit cases** (the original 12 plus seven):
independent 5/0/13-pixel oracle; empty/count diagnostic contract; scaled numeric
range/nonfinite validation; noiseless iterative refinement; all-five distorted
refinement; RANSAC accepted-subset composition; invalid refinement atomicity.
Range coverage includes 3e200/4e200 -> 5e200, repeated Float64'Last RMS, 1e-200
RMS, negative/NaN/infinite errors, and overflowing coordinate subtraction.
Refinement validation covers short/mismatched counts, bad camera/distortion,
nonfinite coordinates and initial pose. Final noiseless max tolerance is 1e-5
pixels; composition allows only 1e-9 pixels RMS numerical worsening.

The actual-shim raw driver adds success/zero and all-five distortion, all seven
required null pointers, wrong object/image depth/channels/Nx1 shapes, counts,
minimum, nonfinite components/points/camera/distortion and complete cleared-output
checks. Fault injection separately exercises status-OK false with zero outputs
and **15** refinement exceptions (three stages times five categories).
Production excludes every `opencv_calib3d_test_*` control.

`run_refinement_faults` is a separate Ada boundary helper (not an AUnit
registration). It checks exact finite Ada pose equality on false and all fifteen
injected exception paths. The fault artifact is linked ahead of production
libraries using a test-only GPR project: object on Linux, libc++-isolated test
dylib on Darwin. Production libraries are never replaced. This
helper runs serially in Linux/macOS ordinary tests and all pinned matrix targets.
Raw production/fault driver variants run both uninstrumented and with Linux
ASan+UBSan, leak detection enabled, no suppressions, actual shim instrumented.
A synthetic false result is only possible in the fault variant; the production
variant tests the genuine native success/error paths, not a claimed native false
fixture that the reviewed iterative implementations do not portably provide.

Python remains 16 tests; C/C++ standalone helpers remain two (header and profile),
plus the existing compiler-derived layout/interchange helper. See Task 002 in
`bootstrap-validation.md` for actual execution evidence; inventory is not a pass.

## Task 003 inventory

The current suite registers **32 AUnit cases**: original 19 plus 13 for zero-
distortion normalized/order oracle, independent all-five Brown inversion,
Rodrigues identity/quarter-turn/orthonormality/determinant, exact/roundtrip point
transforms, direction frames/magnitude/translation invariance, principal/off-axis
camera rays, R^T world-ray orientation, distorted world-ray composition, complete
rotation-record layout/interchange, empty arrays, invalid ray parameters,
invalid transforms/range, and robust bearing normalization at 1e300.

Python inventory is **18**, standalone C/C++ helpers two, shell scripts seven.
Raw actual-shim production/fault tests add undistortion zero/five coefficients,
strided real Core Region, all required nulls, Float32 C2/Float64 C1/C3/wrong shape/
N-D, NaN/infinities, invalid intrinsics/distortion, nonfinite native output,
Rodrigues identity/quarter-turn, invalid poses/outputs and huge finite rotation.
Failed undistortion preserves destination pointer/schema/contents; failed rotation
clears all nine fields. Four checkpoints times five exception classes cover
undistortion/Rodrigues publication and cleanup. The same driver runs uninstrumented
and in production/fault ASan+UBSan. The reused Ada fault helper adds 20 wrapper
exception translations while retaining the 15 refinement scenarios and false case.
Current executed results and cross-version tolerance evidence are recorded in
`bootstrap-validation.md`, not inferred from this inventory.
