# Task 027 qualification and developer source review

## Baseline established from fetched main

Actual starting SHA: `b2a30adc78b6bc2418b58d883c5ef4e6e49daa24`, matching the
expected SHA. Worktree was clean. GitHub confirmed
[PR #27](https://github.com/zackboll/opencv_calib3d_ada/pull/27) merged at this SHA
on 2026-10-18. Created `feature/027-known-plane-homography` directly from fetched
`origin/main`. Read `AGENTS.md`, all six `.clinerules/` files, homography,
relative-camera-motion, world-plane-intersection, plane-correspondence,
reciprocal-plane-correspondence and camera-rays contracts before implementation.

Baseline inspection confirmed 231 AUnit registrations, fourteen manifest examples,
48 matched private C declarations/Ada imports, existing `Homography_Matrix`,
`Map_With_Homography`, relative pose/motion, Rodrigues and direction transforms,
and no public known-pose/known-plane homography constructor.

Actual baseline post-merge hosted conclusions (not inferred from merge):

* [Cross-platform run 39164674330](https://github.com/zackboll/opencv_calib3d_ada/actions/runs/39164674330):
  repository-checks, Linux, macOS, Linux-sanitizers all **success**.
* [Windows post-merge run 39164674250](https://github.com/zackboll/opencv_calib3d_ada/actions/runs/39164674250):
  Windows **success**. No outstanding baseline gate.

## Independent numerical oracles

Ten cohesive new groups give **241** AUnit registrations. All 231 exact baseline
registration expressions remain unchanged, including the multiline expression.

1. Downward Rx(pi) cameras C1=(0,0,10), C2=(2,0,10), plane Z=0:
   independent normalized x2=x1-0.2, y2=y1, including (.1,.2)->(-.1,.2).
2. Independent K1=(1000,1000,320,240), K2=(800,700,300,200): scalar
   (420,440)->(220,340), projectively equivalent [.8,0,-116;0,.7,32;0,0,1].
   Reject nonpositive focal and unrepresentable scaled inverse coefficients.
3. Noncommuting full three-axis rotation vectors (.31,-.22,.14) and
   (-.18,.29,-.24), tilted plane Z=4+.2X-.5Y, three known plane points.
   Test-only scalar axis-angle rotations and perspective equations generate both
   normalized observations independently of production pose/plane/projection APIs.
   Both camera directions are checked as mappings, never equal coefficient scales.
4. Normal sign, positive factor 17 and magnitude 1e300 preserve tilted mapping.
5. Pure rotation about exact common origin and a nonzero common center, including
   a plane containing the center. Scalar Rz(pi/2): (.1,.2)->(-.2,.1).
6. Exact first/second-center degeneracy, 1e-150 near-plane offset, 1e-150 nonzero
   baseline, 1e12 local-world origin, finite inputs with signed-offset subtraction
   overflow, and scale ratio 1e-600 erasing required translation. No epsilon cutoff.
7. Mixed finite/horizon/coordinate-overflow mapping; rebased, empty, single maximum
   bound and three-element arrays ending at Positive'Last; valid general matrix
   with h22=0. No unavailable coordinate is read or treated as a zero success.
8. Default result guards, empty absent-model request, invalid anchor, zero normal,
   invalid rotations/translations, and an invalid late normalized point.
9. Independent all-five Brown projections for both noncommuting cameras; separately
   call `Undistort_To_Normalized` for each calibration, then compare normalized
   transfer (qualified moderate-fixture component tolerance 1e-9).
10. Independent world point (1,-2,0) and scalar image positions crosscheck Task 024
    intersection, Task 025 forward verification and Task 026 reciprocal verification.
    Tight reverse physical range rejects despite correct algebraic homography.

Default normalized analytic tolerance is 1e-11; rectified ideal-pixel scalar
tolerance 1e-9. These are fixture tolerances, not physical accuracy claims or
universal inverse-Brown bounds. The fifteenth synthetic example independently
asserts rectified/different-intrinsics, full rotation/tilted plane and Brown,
pure-rotation, plane-through-center and unavailable projective cases; its observed
maximum mapping residual locally was approximately **1.8874e-15**.

## Native fault and atomicity evidence

Forty new cases: existing stages 3/4 (camera center), 15/16 (Rodrigues),
54/55/56/57 (composition including inversion), all five exception categories.
For every case, an armed hook survives pure-Ada transfer and invalid-normal
prevalidation; valid construction raises `OpenCV_Error`, the previous complete
assigned result compares unchanged, and the following healthy request recovers.
No new native hook, fake pointer, estimator or per-point mapping hook is introduced.

Hooks are first-hit injections: multiple calls to Rodrigues/composition occur in
the reused pipeline. This campaign does not claim independently targeted later
call sites or a dedicated direction-conversion hook. Direction conversion reuses
the same Rodrigues path; algebraic mapping has numerical tests, not invented
native exceptions. Full pre-existing native/Ada campaigns are retained.

## Developer line-by-line production review ledger

This is developer self-review, **not independent source review**. Every changed
production line was reviewed, including declarations, helpers, guards and private
storage. Runtime claims below come from actual campaigns, not source inspection.

| Requirement | Exact source location | Review conclusion |
|---|---|---|
| 1. Relative pose and sign | `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.adb:1067-1071,1095-1107` | Reuses Measure_Camera_Motion; adds t21*n1^T, translation in Camera 2. |
| 2. World normal in Camera 1 | `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.adb:1062,1072` | Existing robust Unit_Vector and World_To_Camera_Direction, no inverse rotation. |
| 3. Signed offsets/degeneracy | `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.adb:1083-1094` | Anchor minus each center, unit world normal, exact zero, guarded absent models. |
| 4. Pure rotation | `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.adb:1077-1080`; `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.ads:186-192` | Existing exact center convention precedes offsets; no depth claim. |
| 5. Projective representation | `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.adb:975-1040,1095-1109` | Finite checked products/sums/differences/quotients; underflow and all-zero rejection; shared max scale. |
| 6. No h22=1 | `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.adb:1019-1029,1107,1127-1134` | Max-absolute coefficient scaling only. Tests include valid h22=0. |
| 7. K order | `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.adb:1114-1138` | Two independent validated K matrices; adjugate(K1); K2*(H*inverse). |
| 8. Distorted pixels prohibited | `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.ads:665-672`; `/home/zboll/git/opencv/calib3d/docs/known-plane-homography-contract.md:112-127` | Explicit ideal-only contract; no distortion term in matrix. |
| 9. Transfer guards/order | `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.adb:1042-1055,1140-1161`; `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.ads:193-214,1484-1487` | Guarded matrices/discriminated coordinates; validate whole batch; subtraction-before-addition positional indexing. |
| 10. Native versus arithmetic failure | `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.adb:1057-1083,1108-1110,1119-1126,1135-1137` | Native calls and accessors outside arithmetic-only handlers; no OpenCV_Error catch. |
| 11. Reuse, no duplicate native kernels | `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.adb:1062-1072,1154-1157` | Existing motion, Rodrigues, direction and public homography mapper; no production Brown/estimator implementation. |
| 12. ABI/dependency/Windows policy | `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.ads:182-214,661-672,1484-1487`; `/home/zboll/git/opencv/calib3d/scripts/check_repository.py:58-81` | Owned Ada additions only, count remains 48. No C++/internal/dependency/Windows workflow changes. |

No changed production body line outside the ledger's 975–1161 range; public spec
changes are exactly 182–214, 661–672 and 1484–1487 plus separating whitespace.
Workflow edits only add the fifteenth example to existing Linux/macOS/manual pinned
steps. No topology, assertions, sanitizer settings or warning switches are weakened.

## Actual local qualification

Linux, OpenCV **4.10.0/calib3d**, Alire **2.1.1**, GNAT **16.1.0**. Alire
build/test/example mutations are serial, including sanitizer campaigns.

| Campaign | Result |
|---|---|
| Production Alire build, warnings as errors | PASS |
| AUnit existing and new groups | 241/241 PASS, zero failed assertions or unexpected errors |
| All fifteen manifest examples | PASS |
| Native production/fault actual-shim boundaries | PASS |
| Complete Ada native-fault campaign | PASS, including all 40 Task 027 cases |
| Production/fault actual-shim ASan/UBSan | PASS |
| Complete leak-enabled Ada actual-shim sanitizer fault campaign | PASS |
| C/C++ numerical and profile drivers | PASS |
| Repository/configuration Python suite | 36/36 PASS |
| Repository inventory, shell syntax, integrity, diff whitespace | PASS |

Initial development feedback caught one missing test-fixture validity suppression
for deliberately injected NaN, corrected to the existing negative-test convention.
Source-regression tests were corrected to match operation calls rather than enum
substrings and Markdown formatting; no production or prior assertion was weakened.

The final exact tested/pushed SHA and hosted/pinned URLs belong in the PR ledger:
embedding this file's own SHA would change it. Re-run qualification on the final
commit and verify clean local/remote/PR head parity. Baseline success is not final
feature qualification. Hosted required jobs: repository-checks/Linux/macOS/
Linux-sanitizers; existing manual matrix: 4.1.0/calib3d, 4.10.0/calib3d,
5.0.0/geometry. Windows remains post-merge only, with its workflow untouched.

Keep the PR open, non-draft, unmerged, auto-merge disabled. No version bump,
amend, force push, merge, release or tag. Stop at independent source-level review.