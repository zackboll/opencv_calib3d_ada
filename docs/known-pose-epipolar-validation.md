# Known-pose epipolar validation (Task 022)

## Baseline evidence (not Task 022 qualification)

Fetched `origin/main`: **2d03d3c36588f028ce786a4376cd36aacbcce459**, matching the
requested SHA; clean worktree. [PR #22](https://github.com/zackboll/opencv_calib3d_ada/pull/22)
merged Task 021 at this commit. Repository integrity confirmed 184 existing AUnit
registrations, 48 private C ABI operations, and nine synthetic example mains.

* Main [cross-platform run 39092851116](https://github.com/zackboll/opencv_calib3d_ada/actions/runs/39092851116):
  repository-checks, linux, macos, linux-sanitizers all **success**.
* Main [Windows run 39092851031](https://github.com/zackboll/opencv_calib3d_ada/actions/runs/39092851031): **success**.
* Task 021 head `ae31afa27edf1351fb02eced8f7b5ede78fa4bb4`,
  [pinned run 39086071610](https://github.com/zackboll/opencv_calib3d_ada/actions/runs/39086071610):
  OpenCV 4.1.0/calib3d, 4.10.0/calib3d, 5.0.0/geometry all **success**.
  This run qualifies that Task 021 head, not its merge SHA or this feature.

## Independent numerical oracles

Nine grouped new AUnit registrations preserve every original test: **193 total**.

1. Rectified identity cameras, t2=(-2,0,0): signed E explicitly
   `[[0,0,0],[0,0,1],[0,-1,0]]`; matching y has exact zero error;
   delta-y=.2 gives `.2/sqrt(2)`, two-point RMS=.1 (1e-14 tolerance).
2. Noncommuting Rz(.4), Rx(-.3): relative R written analytically as
   `[[c,s,0],[-d*s,d*c,-u],[-u*s,u*c,d]]`. Chosen t2 gives relative
   t=(0,-3,4), norm=5. Every signed E coefficient checked against a hand-derived
   matrix at 1e-12. Residual magnitude is not used to certify translation sign.
3. Different intrinsics and distinct nonzero all-five Brown models: independent
   scalar forward projection of noncoplanar points. Every clean normalized error
   <=1e-10 (the established moderate-distortion component qualification scale).
   Reversed camera/calibration/match order accepts; errors agree within 1e-12.
   Separate noncommuting test checks reversed E transpose coefficient by coefficient.
4. Identical cameras, pure rotation about the exact origin center: no E/scores.
   1e-300 and 1e300 baseline retained; unrepresentable norm raises `OpenCV_Error`.
5. Exact zero, nonzero below limits, RMS equality, maximum equality, and each
   limit one binary64 representable step below the measured metric. Equality
   tests use the measured result as a **comparator fixture**, not a truth oracle.
   A gross mismatch has full diagnostics but fails policy. A two-point set with
   one residual near 7.07e159 has RMS 5e159 within 1e-14 relative error, without
   squaring unscaled residuals into overflow.
6. Optical-axis translation with observations at the exact epipole: undefined
   first pair, defined second pair, no partial-set summary. Huge finite normalized
   coordinates naturally overflow existing native metric arithmetic, both directly
   and through the pixel-calibration pipeline; no native behavior is modified.
7. Empty/insufficient guard behavior, unequal lower bounds, singleton and multiple
   elements ending at `Positive'Last`, one-based owned output.
8. Mismatched lengths, NaN/infinity, invalid calibration/Brown/pose/policy fields,
   including validation before empty return. Zero minimum is excluded by Positive.
9. Accepted Task 021 adapter fixtures, rejection of unaccepted inputs even for
   empty matches, result usability after temporary pose values leave scope.
   No fixed RANSAC candidate order or iteration sequence is assumed.

The tenth example independently forward-projects 24 noncoplanar points, checks a
hand-derived signed E for Rz(.2), t=(-2,.3,0), accepts clean observations, rejects
four mismatches and explicitly reports pure-rotation unavailability. Assertions
raise on failure (nonzero exit). These are synthetic, not physical measurements.

## Faults, ownership and sanitizers

The existing Ada fault runner adds **50 synthetic exception cases**: stages
3/4 (camera center), 13/14 (undistortion), 15/16 (Rodrigues), 54..57 (composeRT),
each existing five exception kinds. Every case requires `OpenCV_Error`, verifies
the saved owned result is unchanged after a failed assignment, then immediately
checks a healthy recovery. Armed checkpoints survive valid empty/insufficient
requests; an armed undistortion fault survives exact zero baseline.

`Normalized_Sampson_Error` has **no existing injection checkpoint**. No new C++
instrumentation was introduced. Natural denominator-zero/arithmetic-unavailable
outcomes and existing actual-shim metric boundary tests are distinct from
synthetic exception controls. No Sampson injection coverage is claimed.

The new `run_ada_faults.sh sanitizers` Linux mode instruments the actual fault
shim and links the entire existing Ada campaign with ASan/UBSan and leak detection.
The initial run exposed 1680 leaked bytes in ten existing Essential fault cases:
`Read_Indices` allocated through an `in out` access parameter, then raised before
copy-back. Cleanup in its exception handler now frees the otherwise unreachable
buffer; exceptional controlled return construction also explicitly finalizes owned
buffers. No ABI/native change or suppression was needed. A subsequent leak-enabled
run passed. This additional gate checks temporary Core allocations across all
new pair faults; no fake handles or test-only production pointers are used.
The existing production/fault C++ actual-shim sanitizer drivers remain separate.

## Reproducible local commands

Run Alire commands **serially** (shared generated directories). Local native
backend is OpenCV 4.10.0/calib3d. Warnings remain errors.

```sh
sh scripts/test.sh
alr -n -C examples build
for example in pnp homography fundamental essential triangulation relative_motion metric_triangulation metric_refinement landmark_relocalization known_pose_epipolar; do
    alr -n -C examples exec -- sh ../scripts/run_native.sh "bin/${example}_synthetic"
done
alr -n exec -- sh scripts/run_sanitizers.sh
alr -n -C tests exec -- sh ../scripts/run_ada_faults.sh sanitizers
sh scripts/run_profile_tests.sh
python3 scripts/check_repository.py
python3 -m unittest discover -s tests/configuration -v
for script in scripts/*.sh; do sh -n "$script"; done
git diff --check
```

## Task 022 execution record

Local execution on the feature tree (GNAT 16.1.0, GPRbuild 26.0.1, OpenCV
4.10.0/calib3d), before committing:

| Check | Observed result |
|---|---|
| Production Alire build, warnings-as-errors | PASS |
| AUnit registrations / executed / successful | 193 / 193 / 193; 0 assertions/errors |
| Original baseline registrations preserved verbatim | 184 / 184 |
| Ten synthetic examples, including independent Task 022 assertions | 10 / 10 PASS |
| Actual-shim native production and fault boundaries | PASS |
| Entire Ada fault campaign, including 50 new cases | PASS |
| Actual-shim C++ production and fault ASan/UBSan, leak detection | PASS / PASS |
| Entire Ada fault campaign with actual-shim ASan/UBSan, leak detection | PASS |
| Four C++ numerical profile executables | PASS |
| Python configuration/repository tests | 27 / 27 PASS |
| Repository integrity, private C/Ada ABI count | PASS, 48 unchanged |
| Shell syntax; git diff --check | PASS |

The tenth example's observed clean RMS/maximum were
6.53890470830014e-17 / 2.15887713069019e-16; corrupted set
5.27235461420596e-2 / 1.75072324746616e-1. Its physical baseline was
2.02237484161567 caller world units. Both nonzero-baseline sets had 24/24 defined
errors; clean accepted, corrupted rejected. Pure rotation had zero defined errors,
no E and no summary, `No_Physical_Baseline`. These are observed local synthetic
values, not promised bit-identical cross-version results.

Hosted qualification must name the exact final pushed SHA; the PR execution
record supplies immutable run URLs and head parity without embedding a
self-referential commit hash in this file. An inventory, a configured step, or
Task 021 green checks do not qualify Task 022. Windows workflow policy is unchanged:
post-merge main pushes and PRs changing its own workflow file; no manual trigger.