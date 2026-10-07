# Task 004 — executed homography validation

## Starting state and preserved baseline

- Starting/current `origin/main`: `4f14fa048a551d9a4f098227a7ea7b3ce63787de`,
  Task 003 merge PR #3. Fetch/prune and clean status checked before branching.
- Branch: `feature/004-robust-homography`, created directly from that main.
- Version unchanged: `0.1.0-dev`.
- Core pin unchanged in all roots:
  `4da9d35ea21e1b2efe96296243ea668b488c6326`.

## Gate 0 — Task 003 Windows post-merge final result

Checked final result once: **PASS**, completed including post-action cleanup.
[Run 37257765872](https://github.com/zackboll/opencv_calib3d_ada/actions/runs/37257765872)
ran main SHA `4f14fa048a551d9a4f098227a7ea7b3ce63787de`.

- OpenCV **5.0.0 / geometry**, package opencv5.
- MSYS2 MinGW64 GCC **16.2.0**, configured `mingw64/bin/g++` (not GNAT C++ driver).
- AUnit **32 registered / 32 executed / 32 passed**, 0 failed assertions,
  0 unexpected errors.
- Logs explicitly execute zero/all-five normalized inversion, Rodrigues,
  exact/roundtrip point transforms, direction frames, principal/off-axis rays,
  R^T world rays, distorted world ray, empty/invalid ray/transform and layout cases.
- DLL and `.dll.a` exist; forbidden static Calib3D `.a` absent. Selected geometry,
  Core and Core-shim import-library paths checked to exist.
- PE import evidence: `libopencv_geometry-500.dll`, `libopencv_core_shim.dll`.
- No Windows job was added to PR CI.

## Local qualification — PASS

Linux, OpenCV **4.10.0 / calib3d**, real pinned Core. All Alire root mutation ran
serially. Warnings remain errors. Executed the requested sequence:

```sh
python3 scripts/check_repository.py
python3 -m unittest discover -s tests/configuration -v
sh scripts/run_profile_tests.sh
for script in scripts/*.sh; do sh -n "$script"; done
alr -n build
alr -n -C tests build
alr -n -C tests exec -- sh ../scripts/run_native.sh bin/run_tests
alr test
alr -n exec -- sh scripts/run_sanitizers.sh
alr -n -C examples build
alr -n -C examples exec -- sh ../scripts/run_native.sh bin/pnp_synthetic
alr -n -C examples exec -- sh ../scripts/run_native.sh bin/homography_synthetic
git diff --check
```

Also executed raw production/fault uninstrumented drivers through
`run_sanitizers.sh native` and the reused public Ada fault helper
`run_ada_faults.sh`, including its added four-before-native proof.

| Check | Executed result |
|---|---|
| Production/tests/examples build | PASS |
| AUnit | **40 registered / 40 executed / 40 passed** |
| Failed assertions / unexpected errors | **0 / 0** |
| Python configuration/static tests | **20 executed / 20 passed** |
| Repository checker | PASS; **20 private C declarations/imports**, exact Core pin, ownership/backend/CI checks |
| Standalone C/C++ helpers | **3 PASS**: C header, PnP numeric profile, independent homography classifier |
| Shell syntax | **7 PASS** |
| Raw production / raw fault | **PASS / PASS**, actual shim |
| ASan production / fault | **PASS / PASS**, leak detection enabled, halt-on-error, no suppressions |
| UBSan production / fault | **PASS / PASS**, halt-on-error, no suppressions |
| Ada fault helper | PASS; existing 15 refinement + 20 camera geometry + **30 homography** exception scenarios, existing refinement false |
| Both file-free examples | **PASS** |
| Diff whitespace check | PASS |

The sanitizer compilation instruments **the real `cpp/opencv_calib3d_shim.cpp`**
and raw driver, in production and fault variants. It does **not** claim linked
OpenCV or Core are fully instrumented. Production symbol inspection excludes
all `opencv_calib3d_test_*` controls. Fault injection is evidence of exception
cleanup, not real allocator-exhaustion qualification.

### Geometric evidence

- Clean 24-point independent direct-formula fixture: **24 final inliers**;
  maximum forward mapping error **2.52287751403525e-6 pixels**.
- Gross-outlier fixture: **21 final inliers** observed locally; indices **2, 7,
  15 rejected**. Count is evidence, not an exact cross-version public promise.
- Both public and raw tests independently check **every inclusion and exclusion**
  against final H's forward destination-pixel threshold with original Float64
  points. Synthetic threshold 0.1 pixels, max iterations 2000, confidence 0.999.
- Exactly four rejected by Ada and raw ABI; nondegenerate five-point fixture
  succeeds. Armed native checkpoint remains pending after Ada rejects four and
  is consumed by the next five-point call, proving rejection before native entry.
- Collinear source/destination: real native success-status **no-model** result;
  Found=False, zero count, empty Ada 1..0 inliers, inaccessible cleared native H.
  No forced no-model control added. Pinned versions must execute this fixture
  before declaring it cross-version qualified.
- Hand-authored mapping oracle, **negative common scale invariance**, `w=0`
  point-at-infinity, tiny nonzero denominator, overflow and nonfinite inputs PASS.
- Independent C++ classifier: exact 3-4-5 threshold boundary; robust huge-value
  hypot; subtraction/product/division overflow; tiny denominator; infinity; scale.
- Example: **21 final inliers**, no known outliers survive, sample mapping error
  **1.03763017004866e-6 pixels**.

### Layout and raw failures

Distinct semantic homography record: compiler-observed C/Ada **size 72 bytes,
alignment 8**, all nine offsets **0,8,16,24,32,40,48,56,64** independently compared
using C sizeof/alignof/offsetof and Ada Size/Alignment/Position. C writes every
field and Ada verifies interchange. Options record also compiler-qualified with
all field positions/interchange; rotation tests remain separate.

Raw tests use real Core handles and real Core Region factories for **both**
noncontinuous inputs. They exercise null inputs/options/output/accessor outputs,
wrong Float32 C2 / Float64 C1/C3 / geometry / N-D schemas, mismatch, 0..4 counts,
nonfinite source/destination, invalid iterations/threshold/confidence, all result
accessors/ranges and destruction. Estimation output clears to null before work;
matrix clears all nine doubles and index clears to zero. Six stages times five
exception categories validate atomic publication/clearing in raw and public Ada.

## Remote gates

Ordinary PR CI must retain exactly repository-checks, linux, macos,
linux-sanitizers. Both examples and homography public/raw/fault tests execute on
Linux/macOS. The manual matrix retains WITH_ADE=OFF and runs OpenCV
4.1.0/calib3d, 4.10.0/calib3d, 5.0.0/geometry, including raw, faults, layout,
ASan/UBSan and both examples.

**Remote results are not inferred from workflow presence or local results.**
Final executed run URLs, per-target geometric evidence and final SHA equality are
recorded in the PR review handoff after ordinary CI and the single stable-head
matrix complete. No merge, release, tag, version bump, amend or force-push.

## Limitations

See `homography-contract.md` and immutable `homography-source-review.md`.
This is planar/projective verification, not general 3-D terrain pose. Legacy
OpenCV estimation converts inputs to Float32; final binding classification uses
original Float64. No coefficient equality or h22=1 invariant, exact cross-version
outlier support count, registration uncertainty or navigation accuracy is claimed.
No new dependencies, public estimator selection, image warping or generators.