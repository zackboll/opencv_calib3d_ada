# Bootstrap validation record

## Creation-time checks

This archive was produced in an environment without GNAT, Alire or native OpenCV.
Therefore it makes **no claim** that the Ada project or native shim has built or
that AUnit has run.

Creation-time standalone checks should include:

- `python3 scripts/check_repository.py`;
- Python configuration tests;
- C11 public C-header/layout syntax profile;
- standalone C++17 validation helper;
- shell syntax checks;
- `git diff --check` once placed in Git.

## Task 001 local native qualification

Starting clean `origin/main`: `a2f4936cd8d54dc831774328386ce54995597b44`.
Branch: `feature/001-pnp-bootstrap`. Core pin unchanged across all Alire roots:
`4da9d35ea21e1b2efe96296243ea668b488c6326`.

Local installation: OpenCV 4.10.0, `opencv2/calib3d.hpp`,
`/usr/lib/x86_64-linux-gnu/libopencv_calib3d.so`; backend `calib3d`.
Alire 2.1.1, GNAT native 16.1.0, GPRbuild 26.0.1.

Corrections demonstrated by compilation/runtime/review:

* Ada 2022 bracket array aggregates avoid GNAT 16 obsolescent-syntax errors.
* Independent wide iteration-bound helper retains INT32 preflight without an
  always-false target-dependent comparison warning.
* Pose schema accepts native 3x1 Float64 scalar vectors (the bootstrap incorrectly
  applied `checkVector(3)` to these, causing both PnP tests to error).
* AUnit runner returns failing process status on assertions/unexpected errors
  (the bootstrap runner misleadingly returned zero even with two errors).
* Reject positive Float64 thresholds that underflow to zero in native float.
* Projection validates a temporary matrix before publishing, so injected failure
  after projection cannot mutate caller output.
* Configure checks native library presence and strictly rejects malformed versions.
* CI branch parser checks the entire push branch list, not just its heading.
* Add rotation/five-coefficient projection evidence, robust center/residual checks,
  no-pose evidence, full record interchange and raw invalid-input coverage.
* Run raw ABI variants and the example on macOS as well as Linux; matrix verifies
  backend and exercises the example too. Windows routing is unchanged.

Commands ran serially, all passed:

```sh
python3 scripts/check_repository.py
python3 -m unittest discover -s tests/configuration -v
sh scripts/run_profile_tests.sh
for script in scripts/*.sh; do sh -n "$script"; done
git diff --check
alr -n build
alr -n -C tests build
alr -n -C tests exec -- sh ../scripts/run_native.sh bin/run_tests
alr test
alr -n exec -- sh scripts/run_sanitizers.sh
alr -n -C examples build
alr -n -C examples exec -- sh ../scripts/run_native.sh bin/pnp_synthetic
```

AUnit: **12 registered/executed/passed**, zero failed assertions/unexpected errors.
Python: **16 passed**; standalone C11 header/layout and C++17 profile passed;
all six shell scripts passed syntax validation. Warning-as-error policy retained.
Compiler-derived size/alignment/member offsets and C-written interchange passed
for intrinsics, distortion5 and pose. Raw production/fault variants passed.
Actual Calib3D shim ASan+UBSan production/fault variants passed, leak detection
enabled, halt-on-error, no suppressions, production test-hook symbol exclusion.

Projection fixtures: identity `(20,60)`, translation `(30,20)`, Rz(pi/2)
`(-10,40)`, five-coefficient distortion `(10.03350625,20.1220125)`.
Camera center: Rz(pi/2), t=(1,2,3) gives `(-2,1,-3)` within 1e-10.
Native false fixture returns Found=False, zero inliers, bounds 1..0; Pose raises
OpenCV_Error. No arbitrary failed pose data is published.

Robust AUnit fixture: 20 correspondences, Found=True, 17 observed inliers,
outliers 2/7/15 rejected, valid unique ascending one-based indices, center within
0.05 per component, maximum squared pixel residual `1.03578951140067E-10`.
Exact 17 is recorded evidence, not a required cross-version assertion.

Example observed:

```text
correspondences: 20; found: TRUE; inliers: 17
rvec = (0.100000002096250,-0.0500000206967514,0.0800000383796028)
tvec = (0.200000025435462,-0.0999999960626436,5.99999993827162)
camera center = (-0.514404453109222,-0.470500511296665,-5.96355737761758)
inlier reprojection RMS pixels: 6.12610176455684E-06
inlier reprojection max pixels: 1.03382351079212E-05
```

These are deterministic synthetic fixtures, not geographic/navigation accuracy.
Public API and version unchanged. Source review and hashes cover official peeled
4.1.0/4.10.0/5.0.0 revisions; see source-provenance.json.

## Remote qualification gates

First PR run `37240632034` demonstrated an additional native defect on macOS
with Homebrew OpenCV 5.0.0: deprecated `cv::Mat_` comma initialization failed
`-Werror`. Replaced it with explicit allocated Float64 matrix filling, common to
4/5, without suppressing deprecation warnings. The standalone raw driver treats
upstream include paths as system headers, while retaining warnings-as-errors for
the actual shim and harness (including Apple OpenCV's C11 header extensions).

Second run `37240869907` built on macOS/5.0 but proved the initial collinear
four-point no-pose fixture nonportable: native P3P reported success with a
nonfinite pose, correctly rejected by the shim. Replaced that fixture with 20
deterministic inconsistent correspondences and a 1e-6 pixel consensus threshold.
No production validation or solver policy was weakened.

Third run `37241198690`: macOS/5.0 passed all 12 AUnit tests, but the standalone
raw-boundary driver lacked the Apple SDK sysroot (`cmath`/`limits` not found).
It now passes the same xcrun-derived `-isysroot` as production GPR compilation.

Expected PR jobs: repository-checks, Linux, macOS, linux-sanitizers.
Windows is main-push-only. The manual pinned matrix targets OpenCV 4.1.0, 4.10.0
and 5.0.0. At the local qualification commit these remote gates are **pending**;
their final run IDs, exact SHA and results belong in the PR review record.
Do not infer a pass from source/workflow presence. Windows is not run for this PR.

Limitations: sanitizers instrument this shim, not all linked OpenCV/Core code;
exception injection does not prove real allocator exhaustion; upstream RANSAC
converts correspondences to Float32; exactly four/five bypass robust consensus.
Task 001 did not qualify calibration/refinement, geospatial or broader APIs.

## Task 002 local qualification

Starting clean current `origin/main`:
`86c85b8ed33832ab54ca0a6fec9ba998fab448b6` (Task 001 merge).
Branch: `feature/002-pose-refinement-diagnostics`. Package remains `0.1.0-dev`;
Core pin remains `4da9d35ea21e1b2efe96296243ea668b488c6326` across all roots.
Local native/toolchain configuration remains OpenCV 4.10.0/calib3d, Alire 2.1.1,
GNAT 16.1.0, GPRbuild 26.0.1. No CI topology changes.

### Gate 0: first corrected Windows baseline PASS

[Windows post-merge run 37249720064](https://github.com/zackboll/opencv_calib3d_ada/actions/runs/37249720064)
completed successfully on the starting main SHA. Exact logs report:

* OpenCV **5.0.0 / geometry**, MSYS2 package `mingw-w64-x86_64-opencv-5.0.0-5`.
* MSYS2 MinGW64 compiler package `mingw-w64-x86_64-gcc-16.2.0-4`, driver
  `C:/Users/runneradmin/AppData/Local/alire/cache/msys64/mingw64/bin/g++.exe`;
  the external shim driver is not GNAT's bundled g++.
* Task 001 AUnit **12 registered / 12 executed / 12 passed**, zero assertions
  failed and zero unexpected errors.
* Calib3D DLL and import library exist, static Calib3D archive absent;
  pose/Core/Core-shim import-library existence checks passed.
* Calib3D PE imports `libopencv_geometry-500.dll` and `libopencv_core_shim.dll`.

This qualifies Task 001's corrected baseline, not Task 002's new Windows APIs.
Windows remains main-push-only and is not added to PR CI.

### Local execution results

Every command in the Task 001 serial command block above was re-run for Task 002,
including `alr test` and actual-shim sanitizers, with PASS. Final run:

* repository checker PASS: **12 private C declarations/imports**, **19 AUnit
  registrations**, identical Core pins, ownership/backend/topology checks;
* Python **16 executed/passed**;
* standalone **2 C/C++ helpers** passed (C11 header and C++17 profile), existing
  C compiler-derived record layout/interchange helper passed under AUnit;
* all **7 shell scripts** pass syntax checks, `git diff --check` PASS;
* production/tests/example builds PASS; AUnit **19 registered/executed/passed**,
  **0 failed assertions**, **0 unexpected errors**, both direct and `alr test`;
* raw **production PASS**, **fault-injection PASS**;
* separate public Ada atomicity helper PASS: false plus **15 injected exceptions**;
* actual-shim **ASan+UBSan production PASS**, **fault PASS**, leak detection and
  halt-on-error enabled, no suppressions; no production test-control symbols;
* example PASS, successful refinement on exactly the accepted inlier subset.

Independent diagnostic oracle: errors `(5,0,13)` from known projection `(20,60)`
and observed `(17,56)/(20,60)/(25,72)`. Count=3, maximum=13 pixels,
RMS=`sqrt(194/3)` = **8.04155872120988 pixels**, tolerance 1e-13. Empty summary
and mismatch checks PASS. Scaled large/tiny norms and RMS, nonfinite/negative
errors and overflowing subtraction rejection PASS.

| AUnit fixture | Before RMS px | Before max px | After RMS px | After max px | Center error before | Center error after |
|---|---:|---:|---:|---:|---:|---:|
| noiseless perturbed pose | 3.74137261879513e1 | 4.89771197569171e1 | 4.91991201481758e-13 | 1.02475930687660e-12 | 6.50621748780548e-1 | 4.04780981037361e-14 |
| all-five distortion | 3.76055334982277e1 | 4.91927618262983e1 | 6.58286304724890e-13 | 1.38422770554438e-12 | 6.50621748780548e-1 | 5.56172149813277e-14 |
| RANSAC accepted subset | 6.03028535354324e-6 | 1.01773744718403e-5 | 9.82246697772506e-9 | 2.38666767771226e-8 | 1.18873841546227e-7 | 7.13997687423824e-10 |

Center errors are Euclidean distances in the synthetic caller's world units, not
navigation accuracy. Distortion is `(0.10,-0.04,0.003,-0.002,0.01)`; the perturbed
pose is rvec `(0.16,-0.09,0.12)`, tvec `(0.45,-0.30,6.40)`.
Noiseless final max requires <1e-5 pixels, empirically established locally and
against the changed OpenCV 5 optimizer on macOS; the pinned
matrix must independently establish portability. RANSAC composition permits
only 1e-9 pixels RMS worsening. No bit-identical vectors are required.

Example (17 accepted inliers, recorded rather than required exact count):

```text
RANSAC RMS pixels: 6.12610176455684e-6
RANSAC max error pixels: 1.03382351079212e-5
refinement succeeded: TRUE
refined RMS pixels: 1.32010994715144e-8
refined max error pixels: 2.92692976880911e-8
refined rvec: (0.100000000026355,-0.0500000001446639,0.0800000000225031)
refined tvec: (0.200000000210169,-0.0999999999516296,5.99999999986829)
refined C_world: (-0.514404301070847,-0.470500522183904,-5.96355745106546)
```

The actual native path is `solvePnP(..., true, SOLVEPNP_ITERATIVE)`, not
`solvePnPRefineLM`. All twelve source hashes and the three peeled revisions were
reverified unchanged; see `pnp-source-review.md` and `source-provenance.json`.
Native false is tested using a distinct test-only override after a real solve:
status OK, flag=0, six output components zero, exact Ada initial pose preserved.
Production has no false/exception controls. All five exception categories at
three useful native refinement stages preserve public pose and clear C outputs.

Development feedback caught two new test-helper issues: Float64 operators needed
explicit visibility and the fault shim needed to precede the production static
archive (a dedicated test-only GPR linker-leading switch now does so). GNAT's
`-gnatVa` rejected intentional NaN/infinity fixtures before they could reach the
binding; only those negative-test scopes suppress validity checking. Production
warnings-as-errors and validation remain unchanged. Final validation above has
no failed assertions/unexpected errors.

### Task 002 remote gates

First ordinary run `37251482069` on `9de5b155105115be2fe73afc6b3ab028201b79ec`
passed repository-checks, Linux/OpenCV 4.6.0 and production/fault sanitizers. macOS
OpenCV 5.0 built and passed diagnostics/composition but failed two assertions
(19 executed, 17 passed, two failed assertions, zero unexpected errors): the
initial 1e-7 final-pixel bound was tighter than the changed optimizer's empirical
termination. Exact retrieved logs measured final RMS/max respectively
`6.65377426887021e-7 / 1.39939787850841e-6` (no distortion), and
`1.03358928468298e-6 / 1.72898985794433e-6` (all-five distortion). Center errors
improved `0.650621748780547 -> 3.97447466080833e-8 / 6.15176508863633e-8`.
Composition RMS/max improved `6.03028526269110e-6 / 1.01773740518414e-5` to
`9.87012759033044e-10 / 2.39877702732745e-9`, center to `7.19085126503525e-11`.
The normal corrective commit changes only the empirically justified final bound
to **1e-5 pixels** in Ada/raw tests and this record. Improvement assertions,
validation, warnings and production solver remain unchanged. No blind rerun;
the expensive matrix was not yet dispatched.

Second ordinary run `37251777347` on
`c710ee20cf04a328bc9ff7fd0a5b2de85b14897f` passed repository-checks, Linux and
linux-sanitizers. The supplied completed macOS evidence reports OpenCV 5.0.0 /
geometry, **19 registered/executed/passed**, zero failed assertions/unexpected
errors, diagnostics, all three refinement fixtures and raw production/fault ABI
PASS. The fault shim compiled/linked, but launching `run_refinement_faults` failed
because dyld could not locate `libopencv_calib3d_shim.dylib`. The test's production
project dependency retains a dylib runtime dependency even though the fault object
resolves its calls first; Linux's static production shim hides that issue.

The narrow correction preserves production linkage and the fault-object-first
link order. Only the Darwin fault-helper launch receives `DYLD_LIBRARY_PATH`
with the repository `lib`, resolved Core `lib`, and any existing caller path.
The executable is launched directly with that environment, avoiding an additional
system-shell hop. No production API/library mode, install name, test inventory or
CI topology changes; dyld errors still fail normally. This Linux host cannot
execute Darwin dyld: local helper and full serial validation must pass, and the
new ordinary macOS job supplies the actual runtime qualification.

After this correction, direct local `alr -n -C tests exec -- sh
../scripts/run_ada_faults.sh` PASS (false + 15 exceptions), followed by the full
Task 002 serial command list PASS. Counts unchanged: 19 AUnit registered/executed/
passed, zero failed assertions/unexpected errors, 16 Python tests, two standalone
C/C++ helpers, seven shell syntax checks, raw production/fault and actual-shim
ASan+UBSan production/fault PASS, example PASS. Metrics match the table above.
An isolated shell-routing check of the Darwin branch also PASS with unset and
preexisting `DYLD_LIBRARY_PATH` and paths containing spaces; this verifies
environment construction, not native macOS dyld execution.

Corrected run `37252776615` on `e4be03484371e51ea384db7135f7ec21d89ec550`
again passed repository-checks/Linux/linux-sanitizers. Exact completed macOS logs
show **19/19 AUnit**, zero failed assertions/unexpected errors and both raw ABI
variants PASS. The Calib3D/Core shim lookup is fixed; dyld next reports
`@rpath/libgnarl-16.dylib` missing for the helper. Its inherited relative Ada
runtime rpaths are not valid from this helper output location. A further normal
correction extends only the same Darwin test-launch path with the selected
compiler's `gcc -print-file-name=adalib` and compiler `lib` directories, converted
to absolute paths. This retains all production and fault linkage; it does not
rewrite install names or suppress errors. The compatibility matrix remains
undispatched until ordinary CI is completely green.

Run `37253125008` on `b2a2d29592fefa35e87ccc1338d7c75677f59c0e` fixed the
Darwin dylib lookup (no further dyld missing-library error). macOS again passed
19/19 AUnit, zero assertions/errors and raw production/fault ABI. The Ada helper
then terminated with `libc++abi: terminating due to uncaught exception of type
std::invalid_argument` during injection. Directly placing the Apple-Clang C++
object in GNAT's GNU-libstdc++-linked executable mixes exception personalities;
the raw fault executable, linked by Apple Clang, passes the same barrier tests.

The next test-only correction retains Linux's fault-object-first link behavior.
On Darwin, the same instrumentable actual fault shim object is wrapped in a
libc++-linked test dylib and that artifact resolves before the unchanged
production Calib3D library. Its own Mach-O image keeps native exception catches
bound to the Apple C++ runtime. This is not a production linkage/library-mode
change, does not remove production-project linkage, and does not rewrite any
existing install names. Runtime directories remain local to the helper launch.
All injections remain enabled; the matrix is still undispatched.

Ordinary repository-checks/Linux/macOS/linux-sanitizers and the single stable-head
manual 4.1/4.10/5.0 compatibility matrix are **pending** at this local record.
Exact final run IDs/results belong in the PR review evidence. Never infer remote
PASS from local tests. `WITH_ADE=OFF` remains in the matrix; Windows is unchanged.

Limitations: iterative refinement is local optimization, not robust selection or
global correctness; diagnostics are pixels, not pose covariance/uncertainty.
The >=4 binding contract is deliberately stricter than native three-with-guess.
On exceptional return Boolean out copy-back is not guaranteed by Ada; initialize
the caller flag False if inspecting it after catching an error. Sanitizers cover
the actual Calib3D shim, not all linked Core/OpenCV. False injection is not a
claimed portable real iterative false fixture, and allocation injection does not
prove allocator exhaustion. No broader APIs, version bump, merge or release.

## Task 003 starting gate and Task 002 Windows post-merge

Starting clean fetched main: `2c4df3aa96cff395abb16b7a9f92a5b6201b8172`.
Branch: `feature/003-camera-rays-frame-transforms`. No open PR at start.
Version remains `0.1.0-dev`; all Core pins remain
`4da9d35ea21e1b2efe96296243ea668b488c6326`.

Task 002 Windows post-merge run
[37254729905](https://github.com/zackboll/opencv_calib3d_ada/actions/runs/37254729905)
finished **SUCCESS** on that main SHA (windows job 111589266157). Exact completed
logs were retrieved. This is the first Task 002 19-test Windows baseline:

* OpenCV **5.0.0**, package `opencv5`, backend **geometry**.
* MSYS2 MinGW64 `mingw-w64-x86_64-gcc` **16.2.0-4**, configured driver
  `C:/Users/runneradmin/AppData/Local/alire/cache/msys64/mingw64/bin/g++.exe`.
* AUnit **19 registered/executed/passed**, **0 failed assertions / 0 unexpected
  errors**. Logs explicitly execute diagnostic 5/0/13, empty/range diagnostics,
  noiseless/all-five refinement, RANSAC subset composition and invalid atomicity.
* External Calib3D DLL and import library existence checks passed:
  `lib/libopencv_calib3d_shim.dll` and `lib/libopencv_calib3d_shim.dll.a`.
  All configured pose/Core/Core-shim import-library paths were checked.
* PE imports include **libopencv_geometry-500.dll** and
  **libopencv_core_shim.dll**. MSYS2 compiler-route assertion passed.

No Windows correction was needed. Windows remains push-to-main only; no Task 003
Windows qualification is claimed before merge, and no Windows PR job was added.

## Task 003 local qualification

Local OpenCV **4.10.0 / calib3d**, Alire 2.1.1, selected GNAT 16.1.0 /
GPRbuild 26.0.1. Entire requested command list ran **serially and passed**:
repository checker, Python discovery, C/C++ profile helpers, all shell syntax,
production build, tests build/execution, `alr test` (including raw production/fault
and the reused Ada fault helper), actual-shim sanitizers, example build/execution,
and `git diff --check`.

| Evidence | Result |
|---|---|
| AUnit registered/executed/passed | **32 / 32 / 32** |
| Failed assertions / unexpected errors | **0 / 0** |
| Python configuration/static tests | **18 / 18** |
| Static C exports / private Ada imports | **14 / 14**, exact matching sets |
| Standalone C11/C++17 helpers | **2 / 2** |
| Shell syntax | **7 / 7** |
| Raw actual-shim production / fault | **PASS / PASS** |
| ASan production / fault | **PASS / PASS**, leaks enabled |
| UBSan production / fault | **PASS / PASS**, halt-on-error |
| Ada fault helper | refinement false + 15 exceptions; camera geometry 20 exceptions **PASS** |
| Extended pnp_synthetic | **PASS**, principal/off-axis normalized/camera/world output |

The actual `cpp/opencv_calib3d_shim.cpp` is instrumented. Linked Core/OpenCV are
not claimed fully instrumented. No sanitizer suppressions; no production
`opencv_calib3d_test_*` symbols. Warning-as-error policy retained.

During development, the first new nonfinite fixtures triggered GNAT validity
errors before wrapper execution (30 passed, 0 assertions, 2 unexpected errors).
The tests now use the same local validity-check suppression as existing nonfinite
fixtures. A separate initial helper compile exposed missing visibility of the
Interfaces.C.int comparison operator; a use-type clause corrected it. These were
resolved before the complete passing rerun, not hidden or counted as passes.

| Independent numerical oracle | Observed / required evidence |
|---|---|
| Zero distortion K=(100,200,10,20) | principal=(0,0), off-axis=(1,0), negative=(-0.5,0.25); 1e-14 component tolerance |
| Five-coefficient forward Brown inversion | max component error **6.43929354282591e-15**, tolerance 1e-10 |
| Rodrigues | identity and Rz(pi/2) exact/simple matrix within 1e-14; general rows/columns/R^T R/det within 1e-12 |
| Point transforms | (1,0,0)->(1,3,3), inverse recovers (1,0,0), camera center->zero |
| Point roundtrip | origin/nonaxis/negative/1e6-scale points; max **1.77635683940025e-15**, absolute tolerance 1e-8 |
| Directions | world X->camera Y; camera X->world -Y; changed t invariant; zero and magnitude-two retained; roundtrips pass |
| Principal camera bearing | **(0,0,1)** within 1e-14 |
| Off-axis camera bearing | **(1,0,1)/sqrt(2)** within 1e-14 |
| Off-axis world ray | origin **(-2,1,-3)**, direction **(0,-1,1)/sqrt(2)** within 1e-12 |
| Distorted world ray | **(-0.188144173676719,-0.282216260515079,0.940720868383598)** agrees with analytic (-0.2,-0.3,1)/sqrt(1.13) within 1e-10 |
| Robust normalization | finite normalized coordinates at +/-1e300 give positive camera Z and unit norm within 1e-12 |

The internal nine-double rotation record passed C sizeof/_Alignof/offsetof against
Ada Size/Alignment/every Position, plus C-written values 1 through 9 read in Ada:
**size 72 bytes, alignment 8, offsets 0,8,16,24,32,40,48,56,64**. These are actual
compiler-derived results, not proof from handwritten expected constants.

Raw tests use real Core handles, including a strided Nx1 C2 ROI, N-D input and
all specified negative schemas/pointers/nonfinite parameters. Failure checks
compare destination data pointer/schema/content, and all nine rotation fields
are zero on failure. Four new checkpoints cover all five existing exception
categories before/after native undistortion/Rodrigues (20 scenarios). The qualified
macOS libc++ fault-dylib isolation is unchanged and reused.

All **15** immutable source file SHA-256/Git blob hashes were independently
re-fetched/verified; official peeled tags match the three documented commits.
Explicit common undistortion policy: **COUNT | EPS, 20, 1e-12 pixel epsilon**;
omitted R/P; empty coefficients for zero distortion. See the new source-review
and contract documents for source locations and bounded fixture tolerance rationale.

Ordinary repository-checks/Linux/macOS/linux-sanitizers and the single stable-head
manual 4.1/4.10/5.0 matrix are **pending at this local commit record**. Final run
IDs, logs, per-target counts/oracles and results are recorded in the PR review
evidence. Do not infer remote PASS from local tests. Matrix `WITH_ADE=OFF` and
Windows push-to-main-only topology remain unchanged.

Limitations: iterative undistortion may not converge uniquely for extreme
distortion; the moderate fixture tolerance is not a universal inverse guarantee.
Directions are dimensionless; origins and s use caller units, not necessarily
meters. Rays imply no range/terrain intersection, Earth model, geodetic/ENU/NED
semantics or estimator/fusion policy. No excluded APIs, new dependencies, version
bump, tag, release, merge or auto-merge.
