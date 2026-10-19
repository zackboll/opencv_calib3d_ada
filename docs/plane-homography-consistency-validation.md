# Task 028 qualification and developer source review

## Established baseline

Fetched main SHA **`1c4405d8b555fb75557b24de2968b9a15e28113b`** matched the
expected SHA; the worktree was clean. GitHub confirmed
[PR #28](https://github.com/zackboll/opencv_calib3d_ada/pull/28) merged at that SHA.
Created `feature/028-plane-homography-consistency` directly from fetched main.
Read `/home/zboll/git/opencv/calib3d/AGENTS.md`, all six files in
`/home/zboll/git/opencv/calib3d/.clinerules/`, and all seven requested contracts/
validation documents before implementation. Confirmed the existing constructor,
robust estimator, undistortion, normalized batch transfer and projective mapper.
Baseline inventory: 241 AUnit registrations, fifteen synthetic examples,
48 matching private C declarations/Ada imports.

Actual post-merge main CI, inspected by job (none pending):

* [Cross-platform 39191092154](https://github.com/zackboll/opencv_calib3d_ada/actions/runs/39191092154):
  repository-checks, Linux, macOS and Linux-sanitizers all **success**.
* [Windows 39191092136](https://github.com/zackboll/opencv_calib3d_ada/actions/runs/39191092136):
  Windows **success**. The Windows workflow is untouched for this feature.

## Independent fixtures and coverage

Eleven new groups give **252** AUnit registrations. A source comparison against
fetched main confirms all **241 exact baseline registration expressions** remain
unchanged, unique, and in original order (including multiline registration).

* Downward Rx(pi) camera centers (0,0,10) and (2,0,10), plane Z=0:
  independent scalar normalized `x2=x1-.2, y2=y1`. Twenty-four spread fitting
  points and eight distinct held-out points; qualified fit tolerance 2e-6, not
  coefficient equality or a universal estimator guarantee.
* Three fitting outliers at original positions 2,7,15; independently retained
  holdout; final identities unique, strictly ascending, valid and excluding those
  gross outliers. No exact cross-version consensus cardinality oracle is claimed.
* Empirical-only shifted mapping and a single corrupted held-out destination
  separately demonstrate image-model success versus supplied-geometry disagreement,
  and model agreement versus observation disagreement. All three metrics survive
  individual policy rejection; unaffected neighbors remain valid.
* Noncommuting axis-angle poses, distinct focal/principal points, two independent
  nonzero five-term Brown models, tilted Z=4+.2X-.5Y plane, independent scalar
  Rodrigues/perspective/Brown world-point observation generator. Separate fitting
  and held-out world grids. Only this moderate fixture's tolerance is qualified.
* Exact common-center identity and nonidentity pure rotation, including a plane
  containing the center; first/second-center plane absence and represented-offset
  overflow retain exact Task 027 dispositions and cannot be accepted as geometry.
* Noncollinear exactly-five fitting request, invalid four, native no-model from
  repeated points, and found but insufficient final support. Complete guards for
  absent empirical model, absent metric and unavailable targets.
* Each metric: zero limit rejection for positive error, below-limit acceptance,
  equality acceptance, measured error exactly one Float64 step above the limit
  rejection. These measured boundaries qualify comparison semantics, not accuracy.
* Four differing lower bounds with multiple elements ending at Positive'Last,
  copied results after native temporaries finalized, complete malformed fields,
  zero/tiny/large/overflow residual arithmetic, projective horizons and mixed
  unavailable mappings without losing neighbors.

The sixteenth file-free example prints predicted disposition, empirical success,
final support, held-out count, each available metric, accepted count, aggregate
disposition, and accepted maxima for clean, outlier, wrong-model, bad-holdout and
tilted-Brown synthetic scenarios. Assertions are unconditional checks that raise
on failure, not disabled assertion pragmas. No physical navigation accuracy claim.

## Actual-shim fault reachability

Reused checkpoints, five existing exception kinds each, **80 pipeline cases**:

| Stages | Actual first reached operation |
|---|---|
| 3,4 | First camera-center calculation in known-plane motion construction |
| 15,16 | First Rodrigues in known-plane motion construction |
| 54,55,56,57 | Existing pose composition/inversion path in construction |
| 13,14 | First-camera **fitting** batch undistortion |
| 17,18,19,20 | Robust empirical homography estimation/staging |
| 21 | Native empirical matrix accessor |
| 22 | Native final fitting inlier accessor |

Every pipeline injection requires `OpenCV_Error`, unchanged prior assigned owned
result, and healthy recovery. Armed faults also survive invalid late policy,
late held-out NaN, and mismatched held-out arrays. These prove preflight occurs
before the armed native checkpoint, not just that invalid requests raise.

Ten additional **isolated** second-camera undistortion cases use real second-camera
observations and existing 13/14 hooks, then recover healthily. First-hit hooks
cannot independently target the second fitting or later held-out batches through
this wrapper. The isolated tests are explicitly **not** later-call pipeline
coverage. Found/count accessors contain no injectable checkpoint. No new hook,
fake handle or native mapping fault is invented; transfers are pure Ada numerical
coverage. Existing complete native/Ada fault and actual-shim sanitizer gates remain.

## Developer self-review ledger — not independent external review

Reviewed every changed production line: the public body additions are exactly
`/home/zboll/git/opencv/calib3d/src/opencv-calib3d.adb:1163-1356`; public spec
additions are exactly `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.ads:216-267,726-737,1553-1569`
plus separating whitespace. No existing production line is changed.

| Review requirement | Exact absolute file and lines | Developer finding |
|---|---|---|
| 1. Full pre-native validation | `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.adb:1253-1278` | Counts, all pixels/cameras/Brown/poses/plane and late policy validated before constructor. |
| 2. Normalized domain | `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.adb:1240-1251,1291-1301` | Explicit structural conversion; independent camera calibration; legacy threshold receives normalized units. |
| 3. No data leakage | `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.adb:1291-1302` | Only Fit_First/Fit_Second feed the one estimator; held-out batches remain separate. |
| 4. Camera/plane orientation | `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.adb:1281-1288` | First/second poses and unchanged plane passed directly to Task 027. |
| 5. Controlled-estimate copying | `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.adb:1295-1308`; `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.ads:1562-1569` | Local limited estimate, owned Ada matrix and identities only. |
| 6. Robust minimum/final provenance | `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.adb:1253-1258,1269-1273,1295-1308` | >=5 input; minimum support 4..N; existing final-model identities copied without reinterpretation. |
| 7. Transfer/guards | `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.adb:1163-1206,1302-1322`; `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.ads:1555-1560` | Public batch/map reuse; discriminated targets; independent error guards. |
| 8. Independent metrics | `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.adb:1323-1337` | Every available error computed before threshold precedence. |
| 9. Inclusive policy/aggregate | `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.adb:1338-1352` | Strict > rejects; no epsilon; all heldouts required with support and both models. |
| 10. Failure boundaries | `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.adb:1208-1229,1281-1356` | Only arithmetic helper catches local arithmetic exceptions; no estimator/accessor/OpenCV_Error handler. |
| 11. Whole-array batching | `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.adb:1281-1302,1314-1322` | One construction, four calibrations, one fit and predicted batch outside point loop. |
| 12. No solver/ABI expansion | `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.adb:1163-1356`; `/home/zboll/git/opencv/calib3d/src/opencv-calib3d.ads:216-267,726-737,1553-1569`; `/home/zboll/git/opencv/calib3d/scripts/check_repository.py:58-60` | Ada-only additions; no C++, internal ABI or dependency changes; 48 matched operations. |

Workflow additions only register the sixteenth example in existing Linux/macOS/
manual-pinned steps. No warning, sanitizer, test assertion or topology gate is
weakened. One existing source test is narrowed to its actual transfer function's
end rather than inadvertently swallowing new unrelated function bodies.

## Execution evidence

| Local campaign | Actual result |
|---|---|
| Production warnings-as-errors Alire build | PASS |
| AUnit (241 preserved + 11 new groups) | 252/252 PASS, zero failed assertions/unexpected errors |
| All sixteen synthetic examples | PASS |
| Actual-shim native production/fault boundaries | PASS |
| Complete Ada fault campaigns | PASS, including 80 new pipeline + 10 isolated calibration cases |
| Production/fault actual-shim ASan/UBSan | PASS |
| Complete leak-enabled Ada sanitizer fault campaign | PASS |
| C/C++ numerical/profile checks | PASS |
| Python repository/configuration suite | 38/38 PASS |
| Repository inventory, shell syntax, git integrity, diff whitespace | PASS |

Qualification uses Linux, OpenCV 4.10.0/calib3d, Alire 2.1.1, GNAT 16.1.0.
All Alire build/test/example mutations are serial. Final executed results and
exact tested/pushed SHA are recorded in the PR ledger, together with exact hosted
cross-platform and manually dispatched pinned 4.1.0/calib3d, 4.10.0/calib3d and
5.0.0/geometry run URLs. Baseline results are not feature results. Any pending job
must be reported as pending, not success. Windows remains under post-merge policy.

Developer feedback corrected an overlapping writable-actual warning by copying
the small discriminated target values before metric evaluation, a fixture helper
name, and an overly strict decimal literal equality. No production thresholds or
existing assertions were weakened to obtain a passing result.

This is developer self-review only. Keep the PR open, non-draft, unmerged and
auto-merge disabled; independent external source review is still required.
No amend, force push, release, tag, crate version bump or merge is authorized.