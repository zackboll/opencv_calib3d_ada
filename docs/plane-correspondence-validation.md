# Task 025 qualification and developer source review

## Baseline evidence

Fetched `origin/main` was exactly
`dc6ae06d6a294346734967cc748ec141e62a6c7f`. Worktree was clean before branch
creation. GitHub metadata confirms PR #25 merged at that SHA on 2026-10-18.
Created `feature/025-plane-constrained-correspondences` directly from fetched main.
Inspection found no equivalent existing public plane-constrained two-camera API.
Baseline inventory: 215 AUnit registrations, twelve examples, 48 matched private
C declarations/Ada imports. All 215 baseline registrations remain unchanged.

Actual post-merge workflow conclusions at that main SHA (not inferred from merge):

* [Cross-platform 39144794451](https://github.com/zackboll/opencv_calib3d_ada/actions/runs/39144794451):
  repository-checks, linux, macos and linux-sanitizers all **success**.
* [Windows post-merge 39144794592](https://github.com/zackboll/opencv_calib3d_ada/actions/runs/39144794592):
  windows **success**. Its workflow and policy are unchanged here.

Read the repository entry point, all six `.clinerules/` files, world-plane,
camera-ray, AP3P-reference, three-view and relocalization contracts before changes.

## Numerical and ownership qualification

Eight cohesive new AUnit registrations produce 223 total:

1. Independent analytic downward Rx(pi), A center=(0,0,10), B center=(2,0,10),
   Z=0, P=(1,-2,0). Independently specified pixels A=(420,440), B=(220,340),
   source range=sqrt(105), B depth=10, near-zero residual. Replacing only B with
   (223,344) produces the independent 3/4/5 residual and identical source result.
2. Independent scalar axis-angle Rodrigues and Brown oracles, different intrinsics,
   all five distinct nonzero coefficients per camera, noncommuting rotations,
   plane Z=4+0.2X-0.5Y. Qualified moderate-distortion world tolerance 1e-8 length
   units, target residual tolerance 1e-6 pixels; not universal bounds.
3. Identity source, X+Z=1: exact parallel, near-parallel grazing, behind-ray,
   mismatched target, nonpositive target and clean neighbors retain independent
   dispositions and original positions. Rejected grazing incidence survives.
4. Same physical center C=0, different Rx/Rz rotations, independent projection.
   No stereo baseline or Essential-matrix construction is involved.
5. Exact zero pixel error; below/equal/one-step-above target limit; exact source
   incidence/range/residual gates, adjacent range/incidence limits, nonzero
   represented residual at large origin with measured equality/adjacent limits.
   Measured-error/diagnostic boundaries are comparator fixtures, not truth oracles.
6. B coordinate overflow, individual residual overflow, clean neighbors after
   unavailable points, and actual nonfinite native Brown projection. Native error
   preserves previously assigned complete results; healthy invocation recovers.
7. Empty output 1..0; unrelated lower bounds, multi-element source and target
   arrays ending at Positive'Last, copies/slices, caller-array mutation, default
   and rejected availability guards, no source/no positive-depth survivors, and
   later-image adapter retaining original positions. Adapter validates rejected
   pixels and count mismatch; empty/no-final-acceptance sets are empty.
8. Mismatched lengths and malformed calibration, distortion, pose, plane and all
   policy fields, including NaN and +/-infinity and empty requests. Count beyond
   INT32 is source-reviewed; on this GNAT target Positive itself is INT32-bounded,
   so no impossible multi-billion-element allocation is claimed as a live test.

The thirteenth example independently forward-projects distinct Brown cameras on
a tilted plane, checks three clean matches and a ten-pixel B mismatch, plus a
separate grazing/nonpositive-depth fixture. Expected aggregate counts are
6 original, 5 source-accepted, 5 target-evaluated, 4 target-positive,
1 pixel-error rejection, 3 final accepted. Observed clean maximum component error
locally was 1.44328993201270e-15 world units. Synthetic observations do not establish
physical terrain-registration accuracy.

`/home/zboll/git/opencv/calib3d/tests/plane_correspondence_batching.gdb` uses symbol
breakpoints on the three-clean-point distorted/tilted fixture. Actual observed
counts: one `Unproject_Pixels_Onto_World_Plane`, one
`Evaluate_AP3P_References`, one native `opencv_calib3d_project_points` invocation.
Run through `alr -n -C tests exec -- /usr/bin/gdb -q -batch -x` followed by the
absolute debugger-script and test-executable paths.

## Actual-shim faults and limitations

Forty new end-to-end injections use existing checkpoints 1/2 (target projection),
3/4 (source center), 13/14 (source undistortion), 15/16 (source Rodrigues), all five
existing exception kinds each. Each requires `OpenCV_Error`, no assignment of a
partial array, unchanged previous complete results and healthy recovery. Valid
empty and invalid-target-calibration calls precede the armed healthy call. For
1/2, source-rejected and no-target-positive-depth calls leave projection armed.

The existing Rodrigues checkpoint is **first-hit only**. The wrapper reaches
source Rodrigues first, so it cannot inject specifically at B Rodrigues without
a new hook. No new hook was added. Ten additional **direct target-evaluator**
15/16 cases exercise its Rodrigues containment, atomic assignment and recovery;
these are not claimed as target-only injection through the wrapper.

Actual native projection failure remains whole-batch all-or-error, not isolated
per point. Per-point numerical failures exercise only existing Ada arithmetic
semantics. All old fault campaigns remain enabled. Production/fault boundary
ASan/UBSan and leak-enabled Ada actual-shim campaigns include these cases using
real Core Mats; they do not substitute fake opaque handles.

## Complete changed-production-line developer review ledger

Every changed production line was reviewed: the complete new public declarations
and private record, accessor implementations, verifier and adapter. There are no
other production changes. Paths and ranges below are absolute workspace paths.
This is **developer self-review**, not independent external source assessment.

| Requirement | Exact source ranges | Review finding |
|---|---|---|
| 1. Full validation before native entry | `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.adb:4377-4412`; `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.ads:1240-1249` | Count match/INT32, both pixels/calibrations/distortions/poses, shared Task 024 plane/policy validator, finite nonnegative B threshold all precede source call. Empty follows identical validation. |
| 2. Source pixel/ray/plane frames | `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.adb:4409-4412`; `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.ads:1226-1237` | Single unchanged Task 024 pipeline with A pose/calibration only; Cartesian Xc=R*Xw+t, C=-R^T*t inherited, no interpretation of t as center. |
| 3. Owned source and guards | `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.ads:1250-1265,1281-1287`; `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.adb:4331-4343,4418-4424` | Private value record, full copied source even when rejected; explicit source flag; no caller references or placeholder geometry publication. |
| 4. Only source acceptance supplies projected points | `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.adb:4414-4417,4425-4438,4367-4375`; `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.ads:1266-1269` | Count/compact gate uses public Accepted(Source), coordinates via its guarded accessor only. Final accessor independently requires final acceptance. |
| 5. Compact/original map | `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.adb:4418-4439,4447-4449,4469-4499` | Preserve original one-based I, compact -> I, B subscript First+(I-1). Adapter copies stored original identity, not slice index. |
| 6. B positive depth and pixel policy | `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.adb:4344-4368,4444-4459`; `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.ads:1259-1269` | Evaluator dispositions map distinctly, unavailable error/depth guarded, finite nonpositive depth retained, direct inclusive pixel comparison, no epsilon. |
| 7. Source/target isolation | `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.adb:4409-4459` | B pixels absent from source call; B never changes source result/coordinate, no retry or optimization. |
| 8. Batching and skipped-native cases | `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.adb:4409-4467` | One source batch, exactly one evaluator for nonempty source compact set, one hypothesis; inherited empty/no-positive native skips. Dynamic count evidence above. |
| 9. Native propagation vs numerical unavailability | `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.adb:4377-4467,4344-4366` | No exception handler in orchestration. Only existing evaluator reports numerical unavailability; native exceptions abort complete assignment. |
| 10. No duplicated algorithms or ABI | `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.ads:1226-1272,1281-1287`; `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.adb:4331-4499` | Complete new production surface reviewed. No new import/native call, solver, undistortion, Brown polynomial, Rodrigues math or plane algorithm. Adapter is final-only, validates all later pixels. |

Existing pipeline and ABI implementations are unchanged. `git diff origin/main --`
for the absolute `cpp`, `src/internal`, production/test manifests and Windows
workflow paths is empty. Example registration changes no dependency or pin.
Repository checks enforce reuse and inventory; no warnings-as-errors gate weakened.

## Local execution and hosted publication

Local environment: Linux, OpenCV 4.10.0/calib3d, Alire 2.1.1, GNAT 16.1.0,
GPRbuild 26.0.1. All Alire build/test/example operations execute serially.

The local campaign executed successfully:

| Check | Actual result |
|---|---|
| `sh /home/zboll/git/opencv/calib3d/scripts/test.sh` | Production warnings-as-errors build, 223 AUnit successes, native boundary production/fault variants and complete Ada fault campaign passed. |
| Alire examples build and every manifest-registered executable via `run_native.sh` | All thirteen examples passed, including exact new fixture dispositions/counts. |
| `alr -n exec -- sh scripts/run_sanitizers.sh` from repository root | Production and fault-injection actual-shim ASan/UBSan passed. |
| `alr -n -C tests exec -- sh ../scripts/run_ada_faults.sh sanitizers` | Full actual-shim Ada campaign passed with `detect_leaks=1`, including all 50 new injections. |
| Python repository/configuration suite | 32 tests passed; inventory check: 223 AUnit, thirteen examples, 48 ABI operations. |
| C++ numerical/profile checks | All existing drivers passed; compiler-derived ABI layout tests also remain in AUnit/native campaigns. |
| Shell syntax, repository integrity and `git diff --check` | Passed; no native/dependency/Windows-policy changes. |
| Symbol-breakpoint batching qualification | Three clean candidates: source call=1, evaluator call=1, native projection=1; entire AUnit run succeeded. |

Exact final tested/pushed SHA, hosted workflow URLs and their real conclusions
are recorded in the PR description/comment after execution. They cannot be
embedded as this file's own commit hash without changing that hash. Before final
handoff, rerun qualification at the final committed SHA and verify local/remote/PR
head parity, clean worktree, non-draft/open/unmerged state and auto-merge disabled.
Hosted cross-platform and manual pinned 4.1.0/calib3d, 4.10.0/calib3d,
5.0.0/geometry conclusions must be checked on that exact SHA; do not infer them
from baseline success or configured workflows. Windows remains after merge only.

The [contract](plane-correspondence-contract.md) records numerical/modeling limits.
This task stops at independent source review; no automatic merge, tag, release,
version bump, amend or force-push is authorized.