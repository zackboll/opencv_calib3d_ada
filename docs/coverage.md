# Coverage

## Task 010 calibrated homography decomposition

83 AUnit registrations and 44 private ABI imports/exports. New cases cover
independent pure/general calibrated motions, five signed projective scales,
known-motion representation, translation magnitude, sign pairs, reconstruction,
invalid inputs, and compiler-derived fifteen-field/aggregate interchange.
Raw/Ada fault coverage uses stages 42..45 and explicitly labeled post-native
zero/malformed controls. All five existing examples remain required.
See [validation](planar-motion-validation.md).

Task 008 registers **74 AUnit cases**, preserving all 64 Task 007 cases and
adding ten focused cases: analytic rectified angle, tiny/zero angle, transpose
quarter-turn, antiparallel rays, inverse-frame/translation-scale invariance,
inclusive combined quality flags, low-parallax usable native reconstruction,
noisy native residual thresholds, invalid inputs/public values, and extreme
finite bearings. Existing composition now measures and assesses forty points.
Six C/C++ helpers and 43 private ABI imports/exports remain unchanged.
See [executed quality evidence](triangulation-quality-validation.md).

Task 007 registers 64 AUnit cases and 43 private ABI imports/exports.
Triangulation coverage includes independent both-camera cheirality, rectified,
nonidentity and inverse-frame geometry, noisy diagnostics, pose-inlier composition,
all status mappings and unknown-status rejection, compiler layouts, complete raw
schema/accessor negatives, and 25 raw plus 25 public Ada exception scenarios.
See [qualification evidence](triangulation-validation.md).

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

Out of scope: broad calibration, stereo, essential,
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

## Task 004 inventory

Current inventory: **40 AUnit registrations**, original 32 plus eight homography
cases: independent mapping/negative-scale/point-at-infinity; tiny-denominator,
overflow and all-field nonfinite validation; 24-point clean model; deterministic
gross outliers; real collinear no-model state; 0..4 rejection and five robust
path; nonfinite/mismatched inputs and invalid numeric options; semantically
distinct homography and options C/Ada compiler layout/interchange.

The raw actual-shim driver uses **real Core Region factories for both strided
inputs**, tests all schemas/nulls/options, clean/outlier final-model classification
in both directions, accessor clearing, destruction and collinear no-model. Six
checkpoints times five fault categories add **30 raw fault scenarios** and **30
public Ada translation/cleanup scenarios** to the existing helper, including the
qualified libc++ isolation on macOS. Production/fault versions run raw and
ASan+UBSan, with actual shim instrumentation, leak detection and no suppressions.

A standalone third C/C++ helper tests the independent classifier's exact 3-4-5
boundary, robust 3e200/4e200 hypot, overflow, tiny denominator, projective infinity
and scale. A source-structure regression ensures native masks cannot be substituted
silently. The separate `homography_synthetic` example is run in ordinary
Linux/macOS and manual pinned CI. See `homography-validation.md` for **executed**
counts/results; inventory is not a pass claim.


## Task 005 inventory

Task 005 suite: **48 AUnit registrations**, adding eight fundamental cases:
known-F 0/3-pixel/scale/undefined oracle; large/tiny finite arithmetic and invalid
inputs; noncoplanar direct stereo clean estimation; gross vertical outliers and
final Float64 inclusion/exclusion; 14 rejection/15 true RANSAC; real collinear
no-model; invalid points/numeric profiles; distinct fundamental/options complete
compiler-derived layout and interchange. There are **27** private C exports,
including a shared native options-profile preflight helper, and **four** standalone
C/C++ helpers. Stage IDs 23..28 add 30 raw and 30 Ada fault scenarios while
preserving the existing macOS isolated runtime. Actual-shim production/fault raw
and ASan+UBSan cover both strided Core inputs, schemas/options, 14/15, clean/outlier,
no-model, access/destruction and cleanup. Linux/macOS and the manual pinned matrix
run all three examples. See `fundamental-validation.md` for executed results;
this inventory is not a pass claim.

## Task 006 inventory

Task 006 baseline: **57 AUnit registrations**, adding nine cases: independent
translation-only/known `[t_hat]_x R` normalized Sampson oracle, scale/sign/range/
undefined/nonfinite geometry, clean and gross-outlier Essential plus relative
pose, exact final-E Float64 inclusion/exclusion, five rejection/six native
RANSAC, invalid normalized observations/options, camera-center helper validation,
distinct Essential/relative-pose/options compiler-derived layout and all-field
interchange, and positive-depth points beyond 50 baseline units.

Task 006 baseline: **39** private C declarations/imports and **five** standalone C/C++ profile
helpers. Stage IDs 29..36 add **40 raw and 40 Ada** exception scenarios, reusing
the macOS-safe fault helper. Raw production/fault and ASan+UBSan cover clean,
outlier, both strided Core arrays, 0..5/6, invalid schema/options and all result
access/destruction. Fault variants separately cover no-E and E-found/no-pose
using test-only suppression after real native calls. Armed stage 29 verifies
five rejects before native entry and six consumes the checkpoint. Pose
qualification checks angular R error, R^T R/det, signed t dot/unit norm, and
normalize(-R^T t) camera-center direction, not E coefficient equality.
Ordinary Linux/macOS and manual pinned matrix now run all five examples.
See `essential-validation.md` for executed evidence; inventory is not a pass claim.

## Task 009 inventory

80 AUnit registrations: six new numerical/refinement cases plus the extended
40-point composition. Coverage includes all analytic Jacobian components against
independent finite differences, scaled damped solve residual/sign, strict accepted
prefix monotonicity, real-DLT clean and genuinely noisy seeds, nonidentity fixed
pose, positive depths, low-parallax caller policy, mixed status preservation,
malformed arguments and extreme finite arithmetic. Five existing examples and
six C/C++ profile helpers remain; private native ABI is unchanged at 43.
See [executed refinement evidence](triangulation-refinement-validation.md).
