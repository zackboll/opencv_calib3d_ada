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
No calibration/refinement, geospatial frame or broader public APIs are qualified.
