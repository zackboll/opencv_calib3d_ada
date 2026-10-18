# Task 024 qualification and developer self-review

## Baseline established

Fetched `origin/main`: `b864af155fa26f70aa8285f0d749793daaea8728`, matching the
requested baseline. The worktree was clean. PR #24 was merged at that commit on
2026-10-18, confirmed using GitHub PR metadata and the fetched merge history.
Created `feature/024-world-plane-unprojection` directly from fetched main.

Baseline inventory and actual local execution: 207 AUnit registrations and
207 successful executions, eleven registered synthetic examples, 48 matched
private ABI declarations/imports. The complete baseline `scripts/test.sh` passed.

Latest main workflows at that exact baseline both completed successfully; no
pending jobs were present when inspected:

* [Cross-platform run 39099279659](https://github.com/zackboll/opencv_calib3d_ada/actions/runs/39099279659):
  repository-checks, linux, macos and linux-sanitizers all successful.
* [Windows post-merge run 39099279637](https://github.com/zackboll/opencv_calib3d_ada/actions/runs/39099279637):
  windows successful. Windows workflow/policy is unchanged by this task.

## Independent geometry and boundary tests

Eight new grouped AUnit registrations preserve every existing registration:

* Downward Rx(pi), C=(0,0,10), Z=0: principal pixel -> (0,0,0), s=10,
  incidence=1; pixel (420,440) -> (1,-2,0), s=sqrt(105). Derived directly
  from (x,-y,-1), not the production helper. Pixel bounds end at Positive'Last.
  A separate equality assertion verifies convenience/direct pipeline identity.
* Vertical X=2, C=0, direction proportional to (0.5,0,1): (2,0,4),
  s=sqrt(20). Tilted C=(1,2,3), V=(2,-1,3), A=(4,0,7), N=(1,2,3):
  numerator before normalization=11, denominator=9, lambda=11/9,
  point=(31/9,7/9,20/3), s=11*sqrt(14)/9. Repeat negated and 1e300-scaled N.
* Independent Rz(pi/2), t=(1,2,3), world=(1,0,7) -> camera=(1,3,10).
  Scalar forward Brown projection uses fx=713, fy=839, cx=301, cy=217,
  (K1,K2,P1,P2,K3)=(0.08,-0.02,0.003,-0.004,0.005). Recovery tolerance
  is 1e-8 caller world units for this moderate fixture only.
* Exact parallel, coplanar, at-origin and behind results; almost-parallel nonzero
  incidence; signed distance retained for behind rejection. Sign classification
  precedes angular policy. A measured incidence equality and its adjacent Float64
  threshold test comparator behavior, not independent geometric truth.
* Exact min=max=s=1 and residual=0 accepted; Float64'Adjacent limits reject just
  outside range. At C=(1e16,0,0), V=(-1,0,1), plane X-Z=1e16-2, reconstruction
  loses the unit X decrement and produces a nonzero represented residual.
  Residual limit zero rejects; measured residual equality accepts; one adjacent
  step below rejects. The measured boundary checks are comparator tests.
* Tiny nonzero components and 1e-300 positive distance; huge finite normals,
  directions and plane positions; subtraction, numerator accumulation, division,
  reconstruction addition and residual subtraction overflow; reconstruction
  product underflow; retained stage diagnostics; valid neighboring rays survive
  one numerical failure. Large origins include preserved two-unit offsets and
  rounded-away unit offsets. Finite s times a normalized component cannot
  overflow mathematically; no fake multiplication-overflow fixture is asserted.
* Empty one-based outputs, different lower bounds ending at Positive'Last,
  copied results from temporary arrays, caller input mutation, all four absent
  numeric/coordinate accessor guards, and parallel diagnostic guards.
* Zero/nonfinite normals, nonfinite anchors, zero/nonfinite directions and
  nonfinite origins, every invalid option field (NaN and both infinities),
  negative/zero/reversed ranges and limits. Empty pixel requests also reject
  invalid calibration, distortion, rotation, translation, plane and options.
  There are no paired-count/mismatched-array parameters in this API.

## Native faults and resource qualification

Thirty added SYNTHETIC actual-shim exceptions use existing checkpoints 3/4
(camera center), 13/14 (undistortion), and 15/16 (Rodrigues), five exception
kinds each. Every case arms its checkpoint before a valid empty request; the
following nonempty request must still raise OpenCV_Error. Previously assigned
two-element results remain exactly unchanged; each failure is immediately
followed by successful healthy recovery. No new hook or production C++ change.
The same fault executable is included in the existing leak-enabled Ada
actual-shim ASan/UBSan campaign. Sanitizer evidence covers resource cleanup,
including Core temporary Mats; these calls allocate no Calib3D result handles.

## Source-level developer self-review ledger

This is developer self-review, **not an independent external audit**. Every
changed production source line was reviewed before PR publication. Paths below
are absolute in the qualification workspace. The only changed production source
files are the public Ada specification and body; unrelated native/profile/internal
sources, dependencies and crate versions are unchanged.

| Concern | Exact source paths and lines | Review finding |
|---|---|---|
| 1. Signs, frames, distance units | `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.ads:174-204`; `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.adb:1330-1348` | Anchor-origin numerator, N.D denominator, positive distance follows normalized forward D; distance is caller length, not camera Z. |
| 2. Normalization and dot products | `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.adb:1234-1238,1260-1314,1318-1320` | Existing scaled Unit_Vector (unchanged lines 826-857) for both N and every D; checked scalar products and sequential sums, no unscaled overflowing norm. |
| 3. Exact classification | `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.adb:1330-1348` | Denominator exact zero branches before division; exact numerator zero distinguishes coplanarity. Finite signed distance zero/negative takes precedence over angular policy. |
| 4. Policy precision | `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.adb:1241-1252,1348-1365` | Validation bounds and rejection comparisons are direct; equal incidence/range/residual passes, no epsilon/clamping/default. |
| 5. Representability and large origins | `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.adb:1265-1314,1333-1336,1354-1370` | Checked differences, sums, products, division; nonzero-to-zero underflow unavailable; residual recomputed on represented point; no saturation. Rounding can still lose tangential offsets. |
| 6. Invalid input versus unavailable arithmetic | `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.adb:1234-1253,1315-1321,1369-1371` | Whole-batch validation outside per-ray block; only dedicated local arithmetic exception is caught. No broad OpenCV_Error/others handler. |
| 7. Native error propagation | `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.ads:661-668`; `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.adb:1376-1386` | Existing World_Bearing_Rays called without handler; native exception leaves assignment unchanged. Empty validation/bypass inherited. |
| 8. Bounds and ordering | `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.adb:1261,1318-1325` | One-based outputs; First+(I-1) avoids Positive'Last overflow; normalized directions retain positional pairing. |
| 9. Guards and ownership | `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.ads:193-220,1227-1232`; `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.adb:1331-1332,1341-1343,1359-1365,1388-1429` | Private scalar/value storage only; flags set after completed calculations; position set only after all gates; all absent numeric accessors raise. |
| 10. Scope/dependencies/ABI | `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.ads:174-220,661-668,1227-1232`; `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.adb:1234-1429` | All added production lines reviewed; no C import, solver, projection, native operation or dependency. ABI inventory remains 48. |

## Execution record and publication

Local qualification uses Linux, OpenCV 4.10.0/calib3d, GNAT 16.1.0 and
GPRbuild 26.0.1. Commands run serially for every Alire build/test/example.
The complete local campaign completed successfully before publication:

| Executed check | Actual result |
|---|---|
| `sh scripts/test.sh` (production build, AUnit, native boundary, Ada faults) | PASS; warnings remain errors; 215 run, 215 successful, zero assertions/errors |
| `alr -n -C examples build` and every registered executable via `run_native.sh` | PASS, all twelve examples |
| `alr -n exec -- sh scripts/run_sanitizers.sh` | PASS, production and fault-injection actual-shim ASan/UBSan boundaries |
| `alr -n -C tests exec -- sh ../scripts/run_ada_faults.sh sanitizers` | PASS, actual-shim Ada fault campaign with `detect_leaks=1`, including all 30 new cases |
| `python3 scripts/check_repository.py` | PASS, 215 registrations, twelve examples, unchanged 48-operation ABI and CI topology |
| `python3 -m unittest discover -s tests/configuration -v` | PASS, 30 tests |
| `sh scripts/run_profile_tests.sh` | PASS, all existing numerical/profile drivers |
| `sh -n` on every repository shell script; `git diff --check` | PASS |

All twelve executed examples: `essential_synthetic`, `fundamental_synthetic`,
`homography_synthetic`, `known_pose_epipolar_synthetic`,
`landmark_relocalization_synthetic`, `metric_refinement_synthetic`,
`metric_triangulation_synthetic`, `pnp_synthetic`,
`ray_plane_unprojection_synthetic`, `relative_motion_synthetic`,
`three_view_landmarks_synthetic`, and `triangulation_synthetic`.

The new example's horizontal/tilted analytic assertions, classification assertions,
and diagnostics completed successfully. Existing examples/assertions were not
weakened. `git diff origin/main -- cpp src/internal alire.toml
.github/workflows/windows-post-merge.yml` was empty.

The final execution and hosted run evidence is recorded in the PR at its exact
head SHA; hosted statuses must not be inferred from configured workflows or
baseline success. The PR must remain open, non-draft, unmerged and without
auto-merge for a separate independent source-level review.

Numerical/modeling limitations are authoritative in the
[contract](world-plane-intersection-contract.md): no universal undistortion bound,
near-parallel conditioning, finite intermediate requirements, large-origin
rounding, supplied pose/plane accuracy, and no Earth/terrain interpretation.