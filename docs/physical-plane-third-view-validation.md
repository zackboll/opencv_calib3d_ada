# Task 030 qualification and developer source review

## Verified source baseline

Fetched `origin/main` and verified:

* Main/merge: `8021a405721ab07c5fb4b3daf1f1d62cfa81503a`.
* Exact tree: `bb1991b393bc0512f02619c25e2658e76528a76a`.
* First parent: `b119bce056dea706e9744e85ae8e258c70809aea`.
* Second parent: `66ffc9b88f96250c72cc22966d5a5604185b8055`.
* Merge subject confirms CALIB3D-029 PR #30. Clean starting worktree;
  `feature/030-physical-plane-third-view` created directly from fetched main,
  not reused from a stale feature/release branch.
* Read all repository rules and Tasks 023/026/028/029 contracts, result
  representations, validators, guarded accessors, evaluator, projection and hooks.
* Actual pre-edit baseline: **266/266 AUnit**, all **17/17 synthetic examples**,
  and existing native/Ada fault campaigns passed on Linux OpenCV 4.10.0.

### Exact merged-main hosted status and blocker

[Cross-platform 40437352820](https://github.com/zackboll/opencv_calib3d_ada/actions/runs/40437352820)
was initially queued and subsequently completed **successfully**, including
repository-checks, Linux, macOS and Linux sanitizers.

[Windows 40437352808](https://github.com/zackboll/opencv_calib3d_ada/actions/runs/40437352808)
completed **failure**, before compilation in `Verify Alire-deployed MSYS2 packages`.
The exact failed-job log reports:

```text
error: package 'mingw-w64-x86_64-gcc-libs' was not found
mingw-w64-x86_64-gcc 16.2.0-6
mingw-w64-x86_64-cc-libs 16.2.0-6
mingw-w64-x86_64-libgfortran 16.2.0-6
mingw-w64-x86_64-libquadmath 16.2.0-6
mingw-w64-x86_64-opencv 5.0.0-7
mingw-w64-x86_64-pkgconf 1~3.0.10-1
```

The unchanged Windows workflow unconditionally asks `pacman -Q` for **both**
`gcc-libs` and `cc-libs`. The observed installed package set contains only the
latter, so this inventory check exits 1 and skips build/runtime qualification.
This is a deterministic package-inventory failure, **not a demonstrated geometry
failure and not successful Windows qualification**. No blind rerun was requested.
Its correction needs separately reviewed workflow handling of supported installed
runtime package names, followed by successful Windows execution. This feature
does not change the Windows workflow or widen PR execution. The failed exact-main
Windows run remains an explicit merge-readiness blocker, regardless of feature
Linux/macOS success. Pending baseline CI did not block isolated development.

## Public surface and source review

The [contract](physical-plane-third-view-contract.md) records precedence before
implementation. New public types are `Physical_Plane_Third_View_Options`,
`Physical_Plane_Third_View_Disposition`, `Physical_Plane_Third_View_Holdout_Result`,
its array, and `Physical_Plane_Third_View_Result`. Operations are
`Verify_Physical_Plane_Third_View` and
`Physical_Plane_Third_View_Landmark_Observations`. Accessors provide `Prior_Evidence`,
`Holdout_Count`, `Holdouts`, `Original_Position`, `Prior_Heldout_Accepted`,
`Third_View_Evaluated`, `Disposition`, `Selected_World_Position`,
`Third_View_Disposition`, `Third_Camera_Depth`, `Third_Pixel_Error`,
`Accepted_World_Position`, and per-point/aggregate `Accepted`.

Developer source inspection confirms:

1. Complete count/pixel/calibration/Brown/pose/threshold validation occurs before
   candidate compaction or any native-backed evaluation, including rejected pixels.
2. Only individual Task 029 acceptance selects candidates. Selection calls the
   guarded Task 026 accepted first-camera representative exactly, never diagnostic
   intersections, averaged/reconstructed points or fitted 3-D coordinates.
3. Owned compact world/pixel arrays preserve original positional pairing. Explicit
   compact-to-original mapping restores all diagnostics, retaining one-based
   held-out identities and all rejected observations.
4. Exactly one source call to the existing evaluator, with one supplied pose,
   outside all per-point loops. No eligible candidates means zero evaluator calls;
   otherwise zero or one projection batch depending on positive-depth eligibility.
5. Actual evaluator dispositions survive unchanged, including numerical absence.
   Prior aggregate rejection precedes third-state classification but does not
   suppress eligible diagnostics. It cannot produce accepted points.
6. The accepted-only adapter validates the entire later batch before any gate,
   independently enforces prior aggregate acceptance, exports final individual
   survivors without requiring complete third-aggregate acceptance, and preserves
   original order/positions and exact accepted world values. Empty arrays are 1..0.
7. There is no exception handler around native-backed evaluation: native failure
   aborts return. Missing-value guards raise rather than exposing placeholder data.
8. Production C++, private C/Ada ABI layouts/imports (48 operations), production
   dependencies and pinned Core commit are unchanged. No new native test hook.

This is developer review, **not independent numerical/source approval**. The PR
must remain open, non-draft, unmerged, with auto-merge disabled for that review.

## Deterministic AUnit campaign

**276/276 registrations passed** on Linux OpenCV 4.10.0, with no failed assertions
or unexpected errors. All **266 original registrations** remain byte-identical as
registration expressions, unique and in original order; ten groups are appended.
The existing scalar axis-angle rotation and five-coefficient Brown oracle generates
third observations independently of production Rodrigues/projection. Source-world
coordinates also have an analytic downward-camera oracle.

| Added group | Exercised evidence |
|---|---|
| `Plane_Third_Oracle` | Complete independent calibration/Brown/noncommuting rotation/translation acceptance; complete prior ownership; exact Task 026 representative and independent world oracle. |
| `Plane_Third_Isolation` | Isolated third residual rejection among accepted neighbors; unchanged source point under third-pixel mutation; seven-survivor adapter, original identities, owned later pixels, rejected accepted-world guard. |
| `Plane_Third_Boundary` | Independent 0.5-pixel residual; equality accepts, immediately smaller limit rejects, immediately larger limit accepts; no epsilon. |
| `Plane_Third_Depths` | Positive, zero and negative third depth; missing residual guard; finite native projection followed by unrepresentable subtraction retains numerical absence and healthy neighbor. |
| `Plane_Third_Prior` | Insufficient fitting support with successful individual third diagnostics; one algebraic held-out rejection; one physical-only held-out rejection; all physical candidates rejected; absolute prior export gate and bounded empty output. |
| `Plane_Third_Bounds_Ownership` | All fitting/held-out/third/later bounds differ, including multiple arrays ending at Positive'Last; mutation/finalization of sources and native temporaries cannot alter owned result/copy. |
| `Plane_Third_Invalid` | Final-element invalid pixels, all calibration fields, five Brown coefficients, six pose fields and threshold reject NaN/both infinities; zero/negative focal lengths and threshold; mismatched count. |
| `Plane_Third_Adapter_Invalid` | Complete later batch validation, including final unused pixel under rejected aggregate and mismatched count. |
| `Plane_Third_Guards` | Default result rejects; every absent third/source/accepted diagnostic guard raises. |
| `Plane_Third_Native_Atomic` | Finite input causing invalid native projection output raises, preserves previous complete assignment, and recovers on healthy input. |

## Fault campaign: exact native checkpoints

The existing fault executable first creates complete owned prior evidence, then
arms the existing third-stage checkpoints. Thus these are not ambiguous first
hits inside earlier homography/reciprocal processing:

* **15/16**: immediately before/after native Rodrigues in reference evaluation.
* **1/2**: immediately before/after the single native third-projection batch.
* Each checkpoint exercises all **five existing exception kinds**: **20 faults**,
  previous-result assignment atomicity and healthy recovery after every fault.
* Before each fault is consumed, **21 malformed requests** (420 total) test
  pixels/count/calibration/Brown/pose/threshold, including invalid pixels for an
  entirely rejected prior candidate set. They leave the armed fault untouched.
* A valid no-candidate request leaves every armed checkpoint untouched; nonpositive
  third depth leaves projection checkpoints armed for the following healthy call.

No new call-site coverage is inferred from older first-hit hooks. Checkpoint 2
tests failure after native projection, not a synthetic malformed-output mutation.
Malformed native projection output is separately exercised by the AUnit overflow
fixture. The full existing production/fault campaigns also continue to pass.

## Local qualification ledger

Environment: Linux, OpenCV **4.10.0 / calib3d**, Alire 2.1.1, GNAT 16.1.0,
gprbuild 26.0.1. All Alire build/test/example commands execute serially.

| Executed local campaign | Outcome |
|---|---|
| Baseline AUnit / examples | **266/266**, **17/17 PASS** |
| Feature AUnit / examples | **276/276**, **18/18 PASS** |
| New example | 8/8 accepted; isolated rejection exports positions 1,2,4,5,6,7,8; rejected prior exports zero |
| Development, validation, release warnings-as-errors builds | PASS |
| Full existing actual-shim production and fault native/Ada campaigns | PASS |
| Actual production/fault shim ASan + UBSan | PASS |
| Leak-enabled sanitized Ada fault executable, including new campaign | PASS |
| Python configuration tests | **42/42 PASS** |
| Repository ABI/dependency/registration/example inventory | PASS |
| Independent native numerical profiles, shell syntax, whitespace | PASS |

GNATprove FSF 16.1.0 completed
`--mode=check --output=oneline --output-header -u opencv-calib3d.adb` through the
Alire environment. The invocation header and report are at
`/home/zboll/git/opencv/calib3d/obj/release/gnatprove/gnatprove.out`.
It reports the Task 030 operations/accessors **skipped; SPARK_Mode => Off** and
flow/proof skipped because only legality checking was requested. No proof of the
new orchestration, floating-point safety, native projection or numerical accuracy
is claimed. No applicable new SPARK proof target was introduced.

Session execution logs reside at `/tmp/calib3d-030-*.log`; these are local session
artifacts, not portable hosted evidence. Exact final tested/pushed commit/tree,
PR URL and actual feature-head Linux/macOS and manual OpenCV 4.1/4.10/5.0 results
are recorded in the PR qualification ledger. Pending hosted jobs must never be
reported as success. Local OpenCV 4.10 success alone does not qualify other versions.

## Limitations and required independent review

Review candidate provenance, aggregate trust gates, availability guards, compact
mapping/maximum-bound arithmetic, exception atomicity and scalar-oracle independence.
Confirm the planar degeneracy warnings: no successful PnP invocation substitutes
for depth diversity, uniqueness or general six-DOF observability. Shared systematic
errors and incorrect matches can agree in all views. No ground-truth, statistical
independence, uncertainty bound, DTED/geodesy or navigation-accuracy claim is made.

Merge-readiness remains blocked by the exact merged-main Windows failure above
and by any failing or incomplete required feature-head qualification. Do not merge,
approve, tag, release or submit to an index at this review gate.