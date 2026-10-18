# Three-view landmark qualification validation (Task 023)

## Task 023-R1 correction — native projection batching

Independent source review identified a real gap at reviewed head
`547926d428450a2858c56478990cb1d28e6b4d16`: Task 023 called the Ada reference
evaluator once, but that evaluator called native `Project_Points ([Point], ...)`
inside its reference loop. The original claim of native batching was incorrect.
The historical review/execution ledger below describes that reviewed snapshot;
its passing tests did not establish native batching and do not qualify R1.
The correction uses existing PR #24 and the same feature branch, with a new
normal commit. No public API, native C++/C ABI, dependency or version changes.

### Corrected algorithm and behavior preservation

One checked native Rodrigues matrix per hypothesis is retained. The existing
camera-coordinate arithmetic and transform-failure handler are unchanged. Only
references with finite representable X/Y/Z and Z>0 enter owned compact world,
original-position and saved-depth arrays. After the transform loop ends, a single
native projection batch processes that compact prefix. No eligible references
means no native projection. The subsequent residual loop maps each predicted
pixel to its original observed pixel and checked depth. Per-point residual
overflow publishes `Numerically_Unavailable` with the existing zero placeholders,
without disturbing valid peers. Finite Z<=0 retains `Non_Positive_Depth`, its
actual depth and undefined-error placeholder. No epsilon or pose recomputation.

Native projection remains outside every per-point exception handler; any native
failure aborts the entire evaluator. No partially constructed matrix escapes a
failed extended return/assignment. Compaction arrays are owned Ada locals, with
no new access types or manually released resources. Native Mats remain owned by
Core, borrowed through existing callbacks, and finalized by existing operations.
The added temporary storage is O(reference count) per hypothesis, reused by scope.
Position calculations subtract before adding, including Positive'Last bounds.

`Summarize_AP3P_References`, Task 016 selection, Task 017 refinement, Task 021
relocalization and Task 023 fitting/candidate selection code are unchanged.
All 204 reviewed-head AUnit registration statements are preserved verbatim.
The third pixels remain verification-only; a different projected residual
cannot select or retry a two-view candidate. Native batch/vectorized paths may
round differently from singleton calls; exact cross-version or batch-size bitwise
equality is not promised. Inclusive comparisons remain unchanged, so observations
exactly at thresholds can reflect native rounding differences without an epsilon.

### Source-level correction self-review ledger

This ledger is developer self-review, not the independent reassessment required
after this correction. Ranges below refer to the corrected tree; historical
Task 023 ranges later in this file refer to reviewed head 547926d.

| Changed source / exact ranges | Review findings and evidence |
|---|---|
| `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.adb:1888-1891` | Owned compact world/map/depth arrays and count. Only initialized prefix is consumed. Counts cannot exceed the traversed array length. No raw pointer/native ownership change. |
| Same file `:1914-1927` | Existing unavailable/nonpositive dispositions retained. Positive references append original world point, one-based original position and previously checked depth. X/Y/Z arithmetic at 1829-1855 and 1903-1913 unchanged. |
| Same file `:1928-1935` | Exactly one Project_Points call site, after the transform loop and before residual loop, guarded by nonzero eligible count. Original world coordinates and unchanged pose/calibration passed to existing native wrapper. No handler catches native failure. |
| Same file `:1936-1954` | Map native pixels back to original reference positions. Original third/held-out observations enter residuals only. Existing Pixel_Error arithmetic retained; Constraint_Error only inside each residual calculation maps to unavailable. Saved positive depth published only on valid residual. |
| `/home/zboll/git/opencv/calib3d/tests/src/calib3d_tests.adb:8821-8881` | Four hypotheses, three positive references interleaved with zero/negative-depth references. Independent scalar pinhole expected pixels/depth/errors, not production projection. Different bounds including multi-element Positive'Last; singleton Positive'Last. Summary counts and non-whole-set semantics asserted. |
| Same file `:8883-8906` | Rotated opposing huge coordinates fail checked camera transformation while two valid peers project in one batch. Separate successful batch has a single pixel-residual subtraction overflow and two valid peers with analytic errors 5 and 2. Undefined placeholders remain zero, not fabricated valid residuals. |
| Same file `:8908-8934,8939-8941` | Genuine native nonfinite projection failure leaves saved complete matrix unchanged. Empty and all-nonpositive requests. Three grouped registrations added, all prior 204 retained. |
| `/home/zboll/git/opencv/calib3d/tests/src/run_refinement_faults.adb:1147-1181` | Ten existing native projection faults across a four-hypothesis mixed batch; saved-result atomicity and healthy recovery. Armed faults survive both empty and all-nonpositive requests, proving projection is skipped. Existing actual-shim campaign remains intact. |
| `/home/zboll/git/opencv/calib3d/tests/configuration/test_repository.py:11-26` | Source regression verifies single Project_Points call follows completed transform loop, precedes residual loop, uses compact prefix/map/depth; complements original view-isolation test, which alone did not establish native batching. |
| `/home/zboll/git/opencv/calib3d/tests/reference_batching.gdb:1-76` | Optional debugger measurements at actual C projection entry, without new shim instrumentation. Source breakpoints intentionally match focused test line ranges. No extra production dependency or replacement of sanitizer tests. |
| `/home/zboll/git/opencv/calib3d/scripts/check_repository.py:57-60` | Inventory updated to 207; private ABI remains exactly 48 operations. Eleven examples unchanged. |

Only production body changes are inside `Evaluate_AP3P_References`; public spec,
C++ shim, internal C API, production manifest and crate version are unchanged.
Review identified no remaining correction blocker. The added native batching is
established both by loop-level source review and runtime native-entry counts,
not by counting calls to the enclosing Ada evaluator.

### Native batching measurements

After building the normal tests, the following optional Linux GDB command ran
the complete AUnit suite against the actual production shim:

```sh
alr -n -C tests exec -- gdb -q -batch -x ../tests/reference_batching.gdb bin/run_tests
```

Observed `opencv_calib3d_project_points` entries:

* Four hypotheses, each with three eligible references: **4**, not 12.
* Above plus the singleton Positive'Last evaluation: **5** total.
* Two separate mixed-arithmetic evaluations: **2** total, despite multiple valid
  references in each; unavailable transforms never enter the compact batch.
* Two healthy hypotheses plus the intentionally failing native batch: **3** total;
  subsequent empty and all-nonpositive evaluations add **zero** calls.

Every debugger count assertion passed, and the same execution reported
207/207 AUnit successful, zero assertion failures or unexpected errors.
GDB is supplemental local evidence, not a new build/CI prerequisite. Its source
breakpoints must be maintained if the focused regression procedures move.

### R1 complete local execution

All reproducible campaign commands below were rerun serially on the corrected
tree: Linux, Alire 2.1.1, GNAT 16.1.0, GPRbuild 26.0.1,
OpenCV 4.10.0/calib3d. Warnings remained errors.

| Gate | Observed R1 result |
|---|---|
| Production Alire build | PASS |
| AUnit registrations / executed / successful | **207 / 207 / 207**, zero assertions/errors |
| Reviewed-head registrations preserved verbatim | **204 / 204** |
| All eleven synthetic examples | **11 / 11 PASS** |
| Production and fault native boundaries | PASS / PASS |
| Entire Ada fault campaign, including ten new R1 native faults | PASS |
| Production and fault actual-shim ASan/UBSan, leak detection | PASS / PASS |
| Entire leak-enabled Ada actual-shim ASan/UBSan campaign | PASS |
| C++ numerical/profile executables | PASS |
| Python configuration/repository tests | **29 / 29 PASS** |
| Repository integrity / ABI | PASS / **48 unchanged** |
| Shell syntax / git diff --check | PASS / PASS |
| Supplemental actual native-entry debugger counts | PASS |

Task 013/016/017 AP3P/reference/summary/refinement tests, Task 021 relocalization
and Task 023 source-selection/isolation/rejection tests all ran and passed.
The three-view example retained 14/2 epipolar pass/reject, 1/13 initial/refined
selected candidates, two third-pixel rejections and 12 finally accepted clean
points; original positions and observed maximum coordinate error match the
historical synthetic example below on this local backend. No general bitwise
equivalence is inferred. No warning/test weakening or sanitizer suppression.

New-head hosted cross-platform and pinned OpenCV qualification is recorded in
PR #24 with exact commit SHA, immutable run/job URLs and observed conclusions.
Previous passing runs at 547926d are historical only and do not qualify R1.
Windows policy remains unchanged. Stop at independent review; no merge/release.

## Baseline (not feature qualification)

Fetched `origin/main` at **a01f6a0f0f853e73a25fddb3c400dc236c1d1241**,
matching the expected SHA; clean initial worktree. Feature branch
`feature/023-three-view-landmark-qualification` was created from that commit.
Repository integrity confirmed 193 AUnit registrations, ten synthetic examples,
and 48 private C ABI declarations/imports before implementation.

Task 022 post-merge results on that exact main SHA:

* [Cross-platform 39095235029](https://github.com/zackboll/opencv_calib3d_ada/actions/runs/39095235029):
  repository-checks, linux, macos, linux-sanitizers all completed **success**.
* [Windows 39095235022](https://github.com/zackboll/opencv_calib3d_ada/actions/runs/39095235022):
  windows completed **success**. No baseline workflows pending at inspection.

These results do not qualify Task 023. Windows policy remains unchanged:
post-merge main pushes; PRs only when its own workflow changes; no dispatch.

## Independent oracles and grouped tests

Eleven grouped AUnit registrations preserve all 193 original registrations:
**204 total**. Tests use real native-backed Core Mats, not fake handles.

1. Rectified source cameras, t2=(-2,0,0), normalized (.5,.125)/(.25,.125):
   independently known world (4,1,8), checked within 1e-12. Third Rz(.2),
   t=(.3,-.4,1): scalar trigonometric projection and depth 9.
2. Noncommuting Rz(.3), Rx(-.2), Ry(.15), three intrinsics and three independent
   nonzero all-five Brown models. Scalar rotation and Brown forward projection
   generate observations; world error <1e-9, third pixel error <1e-7. No production
   triangulation/transform generates expected coordinates. Third-only focal
   overflow after successful fitting must propagate a native projection error.
3. Four-track batch: correct, gross epipolar mismatch, equal-y incorrect disparity,
   and third mismatch. The incorrect disparity implies (8,2,16), still passes
   epipolar and stereo policy, but third held-out pixels reject it. Good track and
   all original positions survive.
4. Noisy rectified y=.126/.124 least-squares oracle (4,1,8), all three modes,
   accepted refinement, rejected refined pixel-worsening policy with independently
   accepted original fallback, and required-refinement rejection. Changing only
   third pixels leaves candidate source and every XYZ bit unchanged; no retry.
5. Refinement skipped by caller angle policy: prefer-original fallback versus
   required-refinement rejection. Exact zero-objective no-improvement is separately
   exercised with existing synthetic SVD-output control in the fault runner;
   noiseless native SVD rounding is not assumed universally nonimproving.
6. Positive/zero/negative third depth; third error equality and one representable
   Float64 step above threshold; representable candidate followed by third camera
   sum overflow. Comparator tests reuse measured error solely as a boundary oracle,
   not as an independent projection/world oracle.
7. Unrelated source bounds, multi-element and singleton `Positive'Last`, empty
   arrays, compact adapter ownership and mapping, slicing/rebasing position retention.
8. No epipolar survivors, no metric survivors, initial quality rejection, exact
   epipole mixed with a good pair, and natural native Sampson arithmetic
   unavailability at large normalized coordinates (no invented injection hook).
9. Exact zero, 1e-300 and 1e300 scale, large origin 1e12, metric scaling overflow.
   Existing controlled native output additionally exercises component underflow.
10. Mismatched lengths, all three pixels/calibrations, Brown/pose fields, NaNs and
    both infinities, negative limits, angles, and iteration bound, even with empty
    inputs or disabled refinement. Existing validation tests remain intact.
11. All unavailable result guards, accepted adapter count/finite-pixel validation,
    and no substitution of unavailable/unaccepted world positions.

Qualification tolerances above apply only to specified independent synthetic
oracles, not production acceptance or baseline separation. Production thresholds
are caller-selected inclusive comparisons without epsilon.

## Native exceptions, ownership and sanitizers

The existing actual fault shim exercises **255 additional synthetic native
exceptions**: three source modes x 17 existing checkpoints x five exception kinds.
Checkpoints cover relative motion/composition, camera center/Rodrigues,
undistortion, triangulation and native projection. Every failed whole-operation
assignment leaves an owned previously accepted array unchanged, followed by a
healthy recovery call. An armed fault survives a valid empty request.

The native projection checkpoints are reached first in fitting diagnostics;
the independent third-only overflowing-projection AUnit case above separately
proves error propagation from held-out evaluation. No claim is made that those
injected fitting faults execute specifically in the third projection. Existing
AP3P evaluator campaigns also test its projection exception paths directly.
Sampson has **no** fault-injection checkpoint; only natural undefined metrics
are qualified. No C++ instrumentation, fake success, sanitizer suppression or
replacement of actual-shim tests is introduced.

Existing SVD-output control runs real triangulation before overriding the first
seed: exact (0,0,2) unit seed gives zero objective and tests all three source modes;
1e-300 component at 1e-300 baseline tests metric underflow rejection. These are
synthetic numerical controls, not promises about naturally returned SVD values.

## Reproducible complete local qualification

Run Alire commands serially; generated directories are shared.

```sh
sh scripts/test.sh
alr -n -C examples build
for name in pnp homography fundamental essential triangulation relative_motion metric_triangulation metric_refinement landmark_relocalization known_pose_epipolar three_view_landmarks; do
    alr -n -C examples exec -- sh ../scripts/run_native.sh "bin/${name}_synthetic"
done
alr -n exec -- sh scripts/run_sanitizers.sh
alr -n -C tests exec -- sh ../scripts/run_ada_faults.sh sanitizers
sh scripts/run_profile_tests.sh
python3 scripts/check_repository.py
python3 -m unittest discover -s tests/configuration -v
for file in scripts/*.sh; do sh -n "$file"; done
git diff --check
```

## Historical Task 023 source-level developer self-review ledger (547926d)

This is **developer self-review**, not an independent external assessment, and
does not replace the independent source-level review gate. Paths below are
absolute for the qualification workspace; line ranges identify reviewed code.

| Requirement | Reviewed sources and findings |
|---|---|
| 1. Public invariants/guards | `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.ads:1103-1184`; `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.adb:3858-3923`. Private result owns data. All geometry/residual accessors enforce availability. Accepted coordinate requires final accepted state; selected candidate explicitly differs from final acceptance. Meaningful zero/negative depth retained; no invalid pixel score exposed. |
| 2. Position/overflow | `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.adb:3940-3944,3966-3997,4047-4059,4090-4120`. Equal counts bounded by INT32 before allocation. Counts increment only for array members. Both maps own original one-based positions. `First+(I-1)` avoids Positive'Last addition overflow. Adapter uses stored positions, not rebased subscripts. |
| 3. View isolation | `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.adb:3956-4064`. Screening, P/Q compaction, triangulation/refinement and selection contain no third pixels. Third data validated at entry, read for evaluation only at 4059 after selection finishes. One evaluator batch, no AP3P solve. |
| 4. Provenance/fallback | `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.adb:3998-4046`. Initial-only calls Task 019 once. Other modes call Task 020 once and copy its original. Accepted refined preferred; original independently accepted before fallback; require-refined never substitutes original. No alternate candidate after third rejection. |
| 5. No fabricated geometry | `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.adb:3866-3923,3967-3981,4024-4041,4070-4077,4111-4115`. Missing values remain guarded. Sampson uses existing discriminant. Candidate can be populated only from accepted Task 019/020 geometry. Adapter consumes accepted accessor only. Internal default zeros are inaccessible when unavailable. |
| 6. Numerical contracts | `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.adb:3130-3166,3242-3262,3933-3962,4031-4035,4072-4077`. Shared validations preserve existing finite/angle/iteration conditions exactly. Existing scaled motion/metric conversion/evaluator kernels remain unchanged. Exact-zero baseline via Task 022; no tiny-baseline epsilon. Direct inclusive Sampson and third error comparisons. |
| 7. Native vs numerical errors | `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.adb:3959-3963,3998-4020,4062-4078`. No exception handler surrounds delegated native operations. Existing per-point evaluator dispositions mapped separately. Native errors propagate as OpenCV_Error; no partial return. |
| 8. Ownership/exception safety | `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.ads:1172-1184`; `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.adb:3966-4120`. Arrays and records contain owned Ada values; no new access types, allocations requiring manual release, controlled handles or pointers. Delegated Core/native results finalize before copied diagnostics escape. Extended return is not published when an exception propagates; failed assignment and leak-enabled actual-shim campaign verify this. |
| 9. No duplicate algorithms | `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.adb:3959-4064`. Existing public stages only: Task 022, Task 019/020 and reference evaluator. No duplicate Essential, Sampson, solver, distortion polynomial or refinement kernel. Scalar Brown polynomial exists only in independent tests/example. |
| 10. ABI/dependencies/behavior | Entire production diff: only `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.ads` and `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.adb` changed. No C++/internal C API/production manifest/version changes; 48 operations unchanged. Existing validator conditions and valid behavior preserved. A distinct adapter name avoids introducing ambiguity into existing unqualified aggregate calls. |

Other changed executable/configuration sources reviewed:

* `/home/zboll/git/opencv/calib3d/examples/src/three_view_landmarks_synthetic.adb:1-103`:
  independent scalar axis rotations and Brown projection, clean truth known before
  fitting, explicit wrong-track assertions, accepted coordinates read only guarded.
* `/home/zboll/git/opencv/calib3d/tests/src/calib3d_tests.adb:8407-8834`:
  independent oracles, all modes, third-only native failure, bounds and guard tests.
  All 193 baseline registration statements verified preserved verbatim.
* `/home/zboll/git/opencv/calib3d/tests/src/run_refinement_faults.adb:1070-1147`:
  reuse existing actual fault controls, distinguish SVD controls from natural
  behavior, each native failure followed by healthy recovery, no synthetic Sampson
  checkpoint. Existing campaign unchanged.
* `/home/zboll/git/opencv/calib3d/tests/configuration/test_repository.py:11-26`:
  static orchestration reuse and third-pixel isolation check complements runtime tests.
* `/home/zboll/git/opencv/calib3d/scripts/check_repository.py:57-79`:
  explicit 204-registration and eleven-example inventory, 48-operation ABI retained.
* `/home/zboll/git/opencv/calib3d/examples/alire.toml:9` and
  `/home/zboll/git/opencv/calib3d/examples/opencv_calib3d_examples.gpr:9-12`:
  eleventh main/executable registered; no dependency/version changes.
* `/home/zboll/git/opencv/calib3d/.github/workflows/cross-platform.yml:49-60,115-126`
  and `/home/zboll/git/opencv/calib3d/.github/workflows/opencv-compatibility.yml:65-77`:
  eleventh example added to existing jobs only; no topology/policy change.

Review findings corrected before original publication: avoid adapter overload
ambiguity; retain available nonpositive third depth. This original self-review
missed the per-reference native projection loop. Independent review identified
that blocker; the R1 ledger above records its correction and new qualification.
Geometric/numerical limitations remain those in the contract.

## Historical Task 023 execution record (547926d, not R1 qualification)

Final local qualification completed on Linux with Alire 2.1.1, GNAT 16.1.0,
GPRbuild 26.0.1 and native OpenCV 4.10.0/calib3d. All commands above ran serially;
warnings remained errors. Observed results:

| Gate | Result |
|---|---|
| Production Alire build | PASS |
| AUnit registrations / executed / successful | 204 / 204 / 204; zero assertions/errors |
| Original registrations preserved verbatim | 193 / 193 |
| All synthetic examples | 11 / 11 PASS |
| Production and fault native boundary tests | PASS / PASS |
| Entire Ada native fault campaign, including 255 Task 023 exceptions | PASS |
| Production and fault actual-shim ASan/UBSan, leak detection | PASS / PASS |
| Entire Ada fault campaign with actual-shim ASan/UBSan and leak detection | PASS |
| C++ numerical/profile executables | PASS (homography, fundamental, Essential, triangulation, correction) |
| Python configuration/repository tests | 28 / 28 PASS |
| Repository integrity / private ABI | PASS / 48 unchanged |
| Shell syntax / git diff --check | PASS / PASS |

The eleventh example observed 16 tracks, 14/2 epipolar pass/reject, 1/13 initial/
refined selected fitting candidates, zero third-depth/numerical rejections, two
third-pixel rejections, and 12 final accepted points. Original accepted positions:
1,3,4,6,7,9,10,12,13,14,15,16. Maximum clean world-coordinate component error:
3.28626015289046e-14. These are observed synthetic results, not guaranteed
bit-identical cross-version values or physical accuracy claims.

Hosted evidence must identify the exact tested head SHA and immutable run URLs,
not infer success from configured gates or baseline results. The PR execution
ledger supplies final head/parity without embedding a self-referential commit SHA
in this file.