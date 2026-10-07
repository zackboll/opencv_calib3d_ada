# Task 006 qualification evidence

Starting clean main: `00ea4b4e516ca16f35b81f3f08c7ca25eb1909e3`.
Branch: `feature/006-essential-relative-pose`.
Version remains `0.1.0-dev`; Core remains
`4da9d35ea21e1b2efe96296243ea668b488c6326` in all manifests.

## Gate 0 — Task 005 Windows evidence (not Task 006 qualification)

Recorded final result once: Windows post-merge run
[37558578432](https://github.com/zackboll/opencv_calib3d_ada/actions/runs/37558578432)
**PASS**, exact main SHA above. Completed 2026-10-07T01:50:24Z.

* OpenCV 5.0.0 / geometry, MSYS2 mingw-w64-x86_64 GCC 16.2.0-4.
* AUnit registered/executed/passed 48/48/48; 0 failed assertions, 0 unexpected errors.
* Fundamental clean support 32, max final error `5.12738890026094e-11`;
  outlier support 29; 2/7/15 rejected. Both tests executed.
* 0..14 public rejection / 15 native true-RANSAC permitted test executed.
  This Windows workflow did not run Linux/macOS armed-fault campaign; do not
  confuse the Windows minimum test with that stronger fault evidence.
* Real collinear no-model executed: native success, Found=False, empty `1..0`,
  matrix inaccessible.
* DLL and import-library file checks passed for `libopencv_calib3d_shim.dll`
  and `.dll.a`. PE imports explicitly include `libopencv_geometry-500.dll`
  and `libopencv_core_shim.dll`.
* No Windows PR CI added. This is Task 005 evidence only.

## Local Linux OpenCV 4.10.0 / calib3d

Local runs PASS: production/tests builds, 57 registered/executed/passed AUnit
cases, zero failed assertions/unexpected errors. Independent normalized Sampson
oracle uses hand-authored x-translation E: exact equal-y error 0, displaced y
error `0.2/sqrt(2)`, scales 1/-7/1e6 identical. Independently generated known
nonidentity R and `[t_hat]_x R` validate 40 noncoplanar correspondences; no
projection binding/native projection constructs the fixture.

| Fixture | Essential support | Pose support | Max accepted Sampson |
|---|---:|---:|---:|
| Clean | 40 | 40 | 1.24280308031389e-15 |
| Gross outliers | 37 | 37 | 1.24280308031389e-15 |
| Far (>50 baseline units) | 40 | 40 | 6.94481474590257e-18 |

Clean/outlier rotation angle: 0 at Float64 trace precision, signed translation
dot: 1, camera-center direction Euclidean error `3.81790528747045e-15`.
R^T R max error `1.13007151321147e-15`, determinant approximately +1,
unit direction within 1e-12. Far-point center error `1.24515318800402e-13`.
Test bounds: angle <1e-5 radians, signed dot >1-1e-8, center error <1e-5;
SO(3)/unit checks 1e-12. Counts/coefficient signs are not cross-version contracts.

Every correspondence is checked for EXACT inclusion/exclusion under final-E
original Float64 Sampson <=1e-6. Both index lists valid/unique/ascending, pose
subset. Gross outliers 2/7/15 absent from both. Raw tests repeat geometry with
both inputs noncontinuous real Core Regions. Ada/raw independently reject 0..5.
Armed checkpoint 29 remains after five, six consumes it, later valid call
succeeds. Source review proves six enters registrator subset path.

Raw production/fault campaigns and actual-shim ASan+UBSan production/fault
campaigns PASS, with leak detection and halt-on-error, no suppressions.
Linked Core/OpenCV are not claimed fully instrumented. All stages 29..36 x five
exception kinds (40 scenarios) clear construction/accessor outputs, avoid partial
publication, and clean up. Existing macOS-safe Ada fault helper exercises the
same 40 exception translations to OpenCV_Error.

No-E and E-found/no-pose are explicitly distinguished after REAL native calls
using distinct test-only controls, absent from production symbols. No-E:
success, Found=False, E accessor error, zero Essential/pose support, no pose.
No-pose: Found=True, E and Essential support retained, Pose_Recovered=False,
pose accessor error, empty pose inliers. Repeated/collinear/zero-parallax probes
locally return nonempty E and are not advertised as portable no-model oracles.

Compiler-derived C/Ada layouts separately verify all positions and all-field
C-written interchange: Essential 72 bytes/alignment 8 (9 fields); relative pose
96/8 (12); options 16/8 (2). Raw negatives use no fabricated handles and cover
null arguments/outputs/accessors, 0..5/mismatch, Float32 C2/Float64 C1/C3,
wrong 2-D/N-D schema, nonfinite observations, invalid threshold/confidence,
negative/count index and unavailable matrix/pose. Outputs always cleared.

Private ABI inventory: 39 matched exports/imports. C/C++ profile helpers: 5
(header, PnP profile, homography profile, fundamental profile, Essential profile).
The fourth file-free example prints geometry, support, 2/7/15 survival,
Sampson/angular/direction diagnostics and unknown-scale/navigation warnings.

## Full serial local campaign and remote gates

Source/API correction approved by the user: 5.0 removed the legacy Essential
overload (3d.hpp 1427..1432, five-point.cpp 439..489). Compile-time 5.0 route
explicitly passes fixed literal 1000; 4.x retains the legacy no-maxIters route.
No public Ada/C options field or runtime version dispatch was added. Pinned
actual-shim production/fault builds and raw compiled-profile output qualify
selection of the intended overload, not just a documentation assertion.

Full serial local command campaign PASS: repository checker (39 matched private
exports/imports, 57 AUnit registrations), Python unittest **24/24**, five C/C++
profile helpers, **7/7** shell syntax checks, root build, tests build, direct
AUnit **57/57**, `alr test` PASS, production/fault actual-shim **ASan PASS and
UBSan PASS**, examples build and all four executions, and `git diff --check`.
Separate final raw production/fault and Ada fault commands also PASS. No Alire
roots ran concurrently. Both AUnit runs report **0 failed assertions and 0
unexpected errors**. Production symbol checks reject any test-control exports.

Actual-shim `-fsyntax-only -Wall -Wextra -Wpedantic -Werror` compile probes PASS
against immutable 4.1 headers (SHA-256 verified), local 4.10 installed headers,
and immutable 5.0 installed headers (SHA-256 verified). They prove 4.1/4.10
select the legacy no-maxIters signature and 5.0 selects its required signature
with explicit fixed literal 1000. These compile probes are not full pinned
runtime qualification; that evidence comes from the manual matrix.

Ordinary PR CI and pinned compatibility are not claimed passed merely because
workflow/test definitions exist. Matrix must be dispatched once only after
local/ordinary CI green on a stable pushed head. Required jobs
remain repository-checks, linux, macos, linux-sanitizers; Windows post-merge only.
Pinned matrix remains manual-only with WITH_ADE=OFF.

Limitations: normalized/calibrated coordinates only; unknown positive baseline
magnitude, no metric navigation position, triangulation/decomposition API,
uncertainty or degeneracy guarantee. Native model selection still has Float32
error/mask boundary despite stable final Float64 public classification.