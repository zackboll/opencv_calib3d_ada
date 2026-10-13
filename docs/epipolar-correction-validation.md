# Guarded native epipolar correction — qualification

Local qualification used Linux with OpenCV **4.10.0 / calib3d** (Debian package),
GNAT/Alire as configured by the repository, and Core pin `4da9d35e`. Hosted CI
and the pinned OpenCV 4.1.0 / 4.10.0 / 5.0.0 matrix are reported separately in
the pull request and are **not** claimed here. Cross-version facts below come
from the source-built research libraries in `/tmp/ocv/inst-<version>`, as noted.

## Baseline

* Main `06d1ac4953c19c17a37e2031b64fe7afffb796eb` (Task 014, PR #14).
* Pinned run `38198320642`: success on 4.1.0, 4.10.0 and 5.0.0 (re-verified via `gh`).
* Windows post-merge run `38197635569`: failure before compilation (MSYS2
  `libgcc`/`libstdc++` 16.2.0-5 vs 16.2.0-4). The Windows workflow is unchanged,
  and Windows remains unqualified.

## Local results (serial)

```text
Repository check            PASS: 48 ABI declarations/imports, 128 AUnit registrations
Python configuration        25 tests OK
Profile helpers             4 C++ profile programs PASS (incl. correction_profile_test) + header C test
Shell syntax                7 scripts PASS
Production build            PASS (-Werror, -gnatwe)
AUnit                       registered 128, run 128, passed 128,
                            failed assertions 0, unexpected errors 0 (115 + 13 new)
alr test                    PASS (128/128; native production/fault boundary PASS)
Raw native boundary         production + fault-injection PASS (uninstrumented and ASan/UBSan)
Raw fault injection         20/20 (checkpoints 58..61 x 5 kinds) + 5 post-native output controls
Ada exception translation   20/20 + 5 native-output rejections, inputs unchanged, recovery
Actual-shim ASan+UBSan      production PASS; fault-injection PASS (halt_on_error, leaks on)
Production symbol hygiene   48 opencv_calib3d_ exports; 0 opencv_calib3d_test_ symbols
Examples                    pnp, homography, fundamental, essential, triangulation: all exit 0
git diff --check            clean
```

Sanitizer scope: only the actual Calib3D shim and test driver are instrumented.
The OpenCV and Core libraries are **not** instrumented. This is not a claim that
the native `correctMatches` implementation is memory-safe for every input. No
sanitizer finding or memory-safety defect was observed.

## Observed numerical metrics (4.10.0, AUnit/example output)

| Case | Result |
|---|---|
| rectified (100,20)/(90,24) | (100,22)/(90,22); displacement 2.82842712474619 = √8; bound 4 |
| general rank-two F, 4 pairs | max corrected error 1.52e-14 px |
| nonidentity stereo, noiseless, N=1/6/25 | max displacement 1.02e-13 px |
| nonidentity stereo, controlled noise | error in 1.15–1.24 px → out 2.8e-14 – 2.0e-13 px |
| after fundamental RANSAC (29 inliers) | example: max corrected 1.14e-13 px, RMS/max displacement 0.424 px |
| options record | size 48, alignment 8, six offsets, C→Ada and Ada→C interchange |

## Fixture dispositions

Measured through the actual shim (raw driver) and public Ada API.
4.x/5.0 rows were measured against the source-built libraries with the shim's
own `correction_profile.hpp`; the version-specific assertions are compiled by
`CV_VERSION_MAJOR` / the native version string.

| Fixture | Disposition |
|---|---|
| F, -F, 2F, 0.5F, 1e±40, 1e±100, 1e±300 F (rectified) | ACCEPTED, identical to analytic oracle |
| coefficient range 1e300 with 4e-300 entry; 1e-310 entry | REJECTED_INPUT (normalization underflow) |
| zero / NaN / ±Inf F, nonfinite points | REJECTED_INPUT |
| rank-one F under test policy | REJECTED_CONDITIONING (s2/s1) |
| rank-one F under permissive policy | REJECTED_NATIVE_OUTPUT (disp 54.27 > bound 11.15) |
| s2/s1 = 1e-6 under permissive policy, pair (-30,55)/(12,70) | REJECTED_NATIVE_OUTPUT (all versions) |
| s2/s1 = 1e-6, pair (100,20)/(90,24) | ACCEPTED (bound is necessary, not sufficient) |
| full-rank [1..8,10] (s3/s1 = 0.0113) | REJECTED_CONDITIONING |
| s2/s1 exactly at minimum; s3/s1 exactly at maximum | ACCEPTED; one ulp beyond: REJECTED_CONDITIONING |
| exactly at epipole | REJECTED_INPUT (undefined line) |
| 1e-9 from epipole, line policy 1e-12 (quality 5.7e-14) | REJECTED_CONDITIONING |
| 1e-9 from epipole, line policy 0 | 4.x REJECTED_NATIVE_OUTPUT (0.170 vs 6.4e-10); 5.0 ACCEPTED |
| 1 px from epipole (quality 5.6e-5), policy 1e-12 / 1e-4 | ACCEPTED / REJECTED_CONDITIONING |
| general F, coordinates 1e3 | ACCEPTED |
| general F, coordinates 1e15 | 4.x REJECTED_NATIVE_OUTPUT (NaN); 5.0 REJECTED_NATIVE_OUTPUT (residual 0.61) |
| rectified F, coordinates 1e100, line policy 0 | ACCEPTED (finite scaled diagnostics) |
| repeated / identical correspondences | ACCEPTED, identical outputs / unchanged, zero bound |
| injected NaN, +Inf, residual, displacement, shape after real native call | REJECTED_NATIVE_OUTPUT |
| injected exceptions at 58..61 | NATIVE_EXCEPTION translated to `OpenCV_Error`, output untouched |

No rejected numerical case is classified as memory corruption.

## Boundaries and ownership

* Lower bounds `(7..11)` and `(20..24)`: Count 5, all arrays `1..5`, paired by position.
* Empty/empty: Count 0; invalid F, rank-one F (SVD policy), and invalid options all
  still raise; mismatched empty/nonempty raises.
* Raw: null F/inputs/options/output; Float32 C2, Float64 C1/C3, 1xN and N-D inputs;
  count mismatch; nonfinite F in all nine positions; nonfinite and negative values in
  all six options; output equal to either input handle (rejected, input intact); a
  distinct handle over the same parent storage (snapshot-before-publication
  succeeds, inputs intact); strided Core Regions; recovery after every fault.

## Cross-version note

On the source-built research libraries, 4.1.0 and 4.10.0 produced byte-identical
correction results for every probed fixture. 5.0.0 differs in the documented ways
(near epipole, near rank one, coordinate magnitude) and in last-digit residuals.
The policy is not changed per version. Version-dependent tests assert each
version's documented disposition.
