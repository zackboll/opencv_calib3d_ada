# Task 026 qualification and developer source review

## Verified baseline

Fetched `origin/main` was exactly `ac469e3d4da7ca97b961ae991cf985dae0faec43`.
The worktree was clean; GitHub confirms [PR #26](https://github.com/zackboll/opencv_calib3d_ada/pull/26)
merged at that SHA on 2026-10-18. Created
`feature/026-reciprocal-plane-correspondences` directly from fetched main.
Inspection confirmed no reciprocal API, 223 AUnit registrations, thirteen
manifest-registered examples and 48 matched private C declarations/Ada imports.
All 223 exact baseline registration expressions remain present unchanged.

Read `AGENTS.md`, all six `.clinerules/` files, and the Task 025, world-plane,
camera-ray, AP3P-reference and landmark-relocalization contracts before editing.

Actual workflow conclusions at the baseline SHA (not inferred from merge):

* [Cross-platform 39150695466](https://github.com/zackboll/opencv_calib3d_ada/actions/runs/39150695466):
  repository-checks, linux, macos, linux-sanitizers all **success**.
* [Windows post-merge 39150695499](https://github.com/zackboll/opencv_calib3d_ada/actions/runs/39150695499):
  windows **success**.

PR/main topology remains precisely repository-checks, linux, macos and
linux-sanitizers. Windows workflow is unchanged: main pushes, or PRs changing
its own workflow only, no manual dispatch. Pinned 4.1.0/4.10.0/5.0.0 remains
manual-only. The fourteenth example is registered in Linux, macOS and pinned runs.

## Independent numerical and physical evidence

Eight grouped registrations give **231** AUnit tests:

1. **Analytic reciprocal geometry**: explicitly supplied Rx(pi) downward-camera
   pixels A=(420,440), B=(220,340) recover P=(1,-2,0) independently. B=(228,340)
   gives independently expected P_B=(1.1,-2,0), separation=0.1, forward error=8,
   reverse error=10. P_A remains exactly unchanged when only B pixels change.
   World tolerance 1e-12, pixel residual tolerance 1e-10 for this qualified fixture.
2. **Independent gates**: 15/15-pixel limits accept both directions while 0.05
   world limit rejects; measured equality accepts, one adjacent Float64 step below
   rejects. Measured separation is only a comparator fixture, not a geometry oracle.
   5/15, 5/5 and 15/5 policies prove forward-only, both, reverse-only rejection
   precedence while retaining separation and both complete directional outcomes.
3. **Independent scalar Rodrigues and Brown**: noncommuting axis-angle poses,
   distinct intrinsic matrices and ten independently supplied nonzero coefficients,
   plane Z=4+0.2X-0.5Y. Three independently specified world points projected by
   test-only scalar oracles, not production intersection/projection. World tolerance
   1e-8, target residual limits 1e-6/2e-6 pixels. Swapping every camera argument and
   policy exchanges reconstructed source provenance; separation stays equivalent.
4. **Source isolation**: mixed forward/reverse/both grazing, behind and range
   rejections with clean neighbors, complete source diagnostics, skipped target
   evaluations and no separation from rejected sources. Separate numerical-source
   overflow case is in the numerical group.
5. **Representability, not physical accuracy**: exact zero, 1e-150 nonzero separation,
   1e300 plane scale and representable separation, finite source points whose
   component differences are finite but Euclidean norm overflows, finite points
   whose subtraction overflows, large-origin rounding erasing a displacement,
   mixed accepted/unavailable/accepted batch, source arithmetic failure with a
   valid neighbor. Tests distinguish source/target failure from separation failure.
6. **Same center**: zero translations and distinct Rz/Rx rotations with plane in
   front of both. No Essential matrix or physical stereo baseline gate exists.
7. **Ownership and indexing**: different lower bounds, multi-element arrays ending
   at Positive'Last on either camera, caller mutation, owned directional copies,
   slices/rebased arrays retaining identities, all default-result guards,
   final-only accepted representative and later-image adapter. Rejected later
   pixels are validated; count mismatch, empty and no-accepted sets covered.
8. **Invalid preflight**: lengths, cameras, Brown fields, poses, plane, every field
   in both directional intersection policies, both target thresholds and negative/
   NaN/infinite separation. Malformed empty requests raise. Native INT32 count
   guard is source-reviewed: this target's Positive is itself INT32-bounded; no
   impossible multi-billion-element allocation is claimed as a live test.

All fixture tolerances are Float64 qualifications, not universal inverse-Brown or
physical accuracy bounds. [The contract](reciprocal-plane-correspondence-contract.md)
defines availability, precedence, units, independent provenance and model limits.

The fourteenth example asserts independent synthetic geometry, mismatch,
direction-specific rejection, world-only rejection, grazing, tilted Brown and
same-center cases. Actual summary: 7 tracks, 5 forward accepted, 7 reverse accepted,
6 with both source points, 1 separation rejection, 4 final accepted. The grazing
fixture's reverse direction accepts under its explicitly loose pixel policy;
that never upgrades the forward-rejected reciprocal result. Accepted original
positions print per labeled fixture; no physical-navigation accuracy claim.

## Faults, native allocation safety and batching

Existing actual-shim checkpoints 1,2 (projection), 3,4 (camera center), 13,14
(undistortion), 15,16 (Rodrigues), each of five exception kinds, give **40** new
reciprocal first-hit failures. Valid empty input and invalid reverse intersection
and reverse target-error policies leave the checkpoint armed; the following
healthy-shaped nonempty request must consume it. Each injected failure raises
`OpenCV_Error`, preserves a previously assigned complete array, and recovers on
the next healthy execution. No partial reciprocal result escapes.

Hooks are first-hit only. They establish forward source or forward target entry,
**not reverse-only native injection through the wrapper**. Another **40** injections
exercise the underlying swapped Task 025 operation with distinct cameras. Existing
Task 025 direct target Rodrigues checks remain. No new production hook or C ABI
operation is added. Full native and Ada fault campaigns run both normally and
with the actual shim instrumented by ASan/UBSan, Ada campaign leak detection enabled.

Source checks verify precisely two whole-array Task 025 invocations before the
reciprocal per-result loop, reversed full argument order, preflight, source guards
and arithmetic handler isolation. Existing Task 023-R1/025 native batching remains.

GDB supplemental measurement brackets the two reciprocal calls (normal and swapped)
in the three-point Brown fixture: **4 directional verifier entries, 4 source batch
entries, 4 native `opencv_calib3d_project_points` entries**, not twelve per-point
projections. The complete 231-test run then exits normally. This establishes entry
counts only for that fixture; it does not replace numerical or source review.
Breakpoints use actual linker symbols `opencv__calib3d__verify_plane_constrained_correspondences`
and `opencv__calib3d__unproject_pixels_onto_world_plane` under C expression syntax;
the initial unresolved Ada spelling/printf attempt was discarded, not counted as
measurement evidence.

## Line-by-line developer source review ledger

This is **developer self-review**, not independent external review. Every changed
production line was reviewed, including extraction of the unchanged Task 025
validator. Absolute paths and exact ranges below identify the final source.

| Requirement | Source | Review result |
|---|---|---|
| 1. Complete validation before native entry | `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.adb:4377-4386,4402-4413,4565-4590` | Unchanged Task 025 policy conditions/error text extracted; both policies and every request field validated before either direction; empty returns afterward. |
| 2. Forward/reverse order | `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.adb:4591-4598` | A->B and B->A each exactly once, complete batches, respective policy. |
| 3. Separate calibration/distortion ownership | `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.adb:4578-4583,4593-4598` | Independent validation and reversed camera/Brown argument ownership, no reused A calibration in reverse source. |
| 4. Original-position mapping | `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.adb:4600-4610,4673-4681` | One-based I copied with full directional records; adapter subtraction-before-addition pairing and stored identity survive slices/rebase. |
| 5. Both-source guards | `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.adb:4524-4526,4604-4615` | Task 024 Accepted predicates gate point reads; rejected sources never contribute separation. |
| 6. Safe subtraction and norm | `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.adb:640-653,703-743,4613-4629` | Guarded positions outside handler; only existing checked subtraction and scaled norm inside. No unscaled squares, saturation or epsilon. |
| 7. Separation despite target rejection | `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.adb:4611-4632` | Evaluate when both source intersections accepted before testing directional target acceptance. |
| 8. Deterministic precedence | `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.adb:4631-4650` | Both/forward/reverse source, both/forward/reverse direction, unavailable, limit, accepted. Full original records retained. |
| 9. Final coordinate provenance | `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.adb:4535-4554,4677-4681` | Both independent source positions guarded; final first-camera representative only in adapter; no averaging. |
| 10. Native propagation and assignment atomicity | `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.adb:4591-4600,4613-4629,4652-4655` | Both fallible native directional stages precede return construction; only arithmetic has a local handler, no native/guarded access inside it. |
| 11. No per-point native projection | `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.adb:4414-4473,4591-4601` | Reuse Task 025 one-source/one-evaluator batch path; reciprocal delegation outside per-result loop. |
| 12. No duplicated kernel/ABI/dependency | `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.ads:1274-1316,1332-1338`; `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.adb:4377-4386,4413,4507-4686` | Entire new production surface is owned Ada orchestration/accessors/adapter. No solver, Brown, ray, pose or intersection kernel added. |

`git diff origin/main -- cpp src/internal alire.toml tests/alire.toml
.github/workflows/windows-post-merge.yml` is empty. Exactly 48 ABI operations
remain. No warning gate, dependency, version, Core pin or Windows policy changed.

## Actual local execution and final-head publication policy

Local Linux environment: OpenCV **4.10.0/calib3d**, Alire **2.1.1**, GNAT **16.1.0**,
GPRbuild executable reports **26.0.0 (2026-04-15)**. Alire mutations run serially.

| Campaign | Actual local result |
|---|---|
| `sh /home/zboll/git/opencv/calib3d/scripts/test.sh` | Production warnings-as-errors build; all 231 AUnit tests pass; native production/fault boundaries and complete Ada fault campaign pass. |
| Alire examples build, all fourteen manifest executables via `run_native.sh` | All pass, including new geometric, disposition and summary assertions. |
| `alr -n exec -- sh scripts/run_sanitizers.sh` | Production and fault actual-shim ASan/UBSan pass. |
| `alr -n -C tests exec -- sh ../scripts/run_ada_faults.sh sanitizers` | Complete leak-enabled actual-shim Ada fault campaign passes, including 80 new cases. |
| Python repository/configuration | Repository inventory check and 34 configuration/repository tests pass. |
| `sh scripts/run_profile_tests.sh` | All C/C++ numerical/profile drivers pass; compiler-derived ABI layout checks remain in AUnit/native campaigns. |
| Shell syntax, `git fsck --no-dangling`, `git diff --check` | Pass. |
| GDB three-point normal/swapped reciprocal fixture | 4 directional calls, 4 source batches, 4 native projection batches; all 231 AUnit tests pass. |

Exact final tested/pushed SHA, hosted URLs and conclusions belong in the PR ledger
(embedding this file's own commit hash would change that hash). Repeat qualification
at the final committed SHA, verify clean local/remote/PR head parity, and check
hosted repository-checks/Linux/macOS/sanitizers and manual pinned
4.1.0/calib3d, 4.10.0/calib3d, 5.0.0/geometry on that exact head. Baseline hosted
success is not a final-head qualification claim. Windows remains after merge only.

The PR must stay open, non-draft, unmerged, auto-merge disabled. No amend,
force-push, merge, tag, release or crate version bump. Stop at independent review.