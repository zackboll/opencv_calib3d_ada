# Task 021 validation and review evidence

## Baseline

Fetched `origin/main` was `9c26704e8a7fcf40c899a566ef1ae7368625a609`, matching the
requested SHA, with a clean worktree. Task 020 PR
<https://github.com/zackboll/opencv_calib3d_ada/pull/21> is merged at that SHA.
The feature branch was created from that fetched main. Baseline execution passed
177/177 AUnit cases with zero failures/errors; repository checks confirmed 48
private ABI declarations/imports.

Actual Task 020 post-merge status was checked, not inferred:

* Cross-platform <https://github.com/zackboll/opencv_calib3d_ada/actions/runs/39084413318>:
  completed success (Linux, macOS, repository-checks, Linux sanitizers).
* Windows <https://github.com/zackboll/opencv_calib3d_ada/actions/runs/39084413264>:
  completed success.

## Qualification design

184 AUnit registrations preserve every original 177 registration. Seven grouped
cases exercise the new operation and adapter:

1. Independent scalar analytic Rz(.3), nonzero t, pinhole plus all five nonzero
   Brown terms; 30 Cartesian landmarks with exact and noisy observations and
   deterministic gross mismatches. Rotation is checked as a physical matrix,
   translation against analytic t, and center against analytic `-R^T*t`. Native
   consensus count is measured, not hardcoded. Every returned position is checked
   for sorted uniqueness, range and exclusion of gross mismatches. A separate
   manual call to iterative refinement on precisely those pairs reproduces the
   accepted pose and per-inlier diagnostics. Zero distortion is also exercised.
2. Zero through five fitting pairs (no pose/refinement), six-point robust entry,
   deliberately inconsistent observations with no native model, and a caller
   support minimum greater than the available native consensus. Rejected result
   accessors are guarded, initial pose retained when present.
3. Measured final inlier RMS, held-out RMS, held-out maximum, and positive RMS
   worsening are each replayed as inclusive equality boundaries, then reduced by
   one representable step to demonstrate rejection despite native refinement
   success. The worsening comparator deliberately uses projections of the initial
   pose, **not** an independent truth oracle. Existing Task 017 scalar policy
   regressions remain, including the scaled-subtraction defect fixture.
4. A held-out point behind the camera, finite inputs whose transformed camera
   coordinates overflow, and a mixed front/back native consensus. The latter
   places holdout in front of the actual initial native camera rather than assuming
   a particular EPNP orientation; final consensus depth must still be checked.
5. A held-out point constructed between initial/refined camera half-spaces tests
   `Refined_Holdout_Unusable` without modifying the native solver. The initial
   summary and valid final consensus diagnostic remain available.
6. Invalid counts, empty holdout, exact duplicate world points and cross-set
   overlap, NaN/infinities in observations/calibration/options, negative limits,
   minimum support below four, and validation before insufficient-fitting return.
   Test-local validity suppression permits deliberate IEEE invalid arguments;
   production validation is not suppressed or weakened.
7. Actual Task 020 accepted/unaccepted values, no candidate/original fallback,
   empty and all-skipped sets, invalid pixels even for skipped landmarks, extreme
   differing bounds including `Positive'Last`, compaction, and end-to-end robust
   relocalization from the compacted map. Native inlier positions map through the
   adapter back to original landmark positions.

The new Ada fault campaign reuses existing native hooks, labeled SYNTHETIC:
native iterative false preserves initial pose/summary and exposes no accepted
pose; all five exception kinds at 12 existing stages cover RANSAC allocation and
publication, rotation, projection, iterative refinement and final camera center
(60 exception/assignment-atomicity checks), followed by recovery. Armed RANSAC
checkpoints prove four/five fitting pairs never enter native estimation. No C++
test hook or production ABI operation was added.

The ninth file-free example is `landmark_relocalization_synthetic`. Two known map
cameras explain the common Cartesian frame; trusted synthetic landmarks are
supplied directly, rather than claiming map accuracy. A third camera has an
independently defined Rz/t truth. Analytic scalar Brown projection generates its
observations, gross mismatches affect fitting only, and six disjoint held-out
landmarks verify the result. It asserts consensus exclusion and physical rotation,
translation and center agreement, raising on failure for nonzero exit status.
These are synthetic observations, not physical sensor or navigation evidence.

## Execution and source review

Local environment: Linux, Alire 2.1.1, GNAT 16.1.0, OpenCV 4.10.0/calib3d.
Complete final qualification evidence is recorded on the review PR with exact
head SHA and hosted URLs. Inventory and planned commands are not pass claims.
Local logs are under `/tmp/calib3d-021-qualification/`.

Complete local qualification passed: production build; 184/184 AUnit executions
with zero failed assertions or unexpected errors; all nine examples; production
and fault-injection native boundary drivers; existing Ada exception/cleanup
campaigns and the added 60 relocalization fault cases; actual-shim production and
fault ASan+UBSan; all C++ profile tests; 26/26 Python configuration tests;
repository integrity/48-operation ABI checks; shell syntax; and `git diff --check`.
Hosted results remain separate and are reported against the exact PR head.

```sh
sh scripts/test.sh
alr -n -C examples build
# Run every one of the nine bin/*_synthetic via scripts/run_native.sh.
alr -n exec -- sh scripts/run_sanitizers.sh
sh scripts/run_profile_tests.sh
python3 -m unittest discover -s tests/configuration -v
python3 scripts/check_repository.py
for script in scripts/*.sh; do sh -n "$script"; done
git diff --check
```

Alire commands run serially. Actual-shim production/fault ASan+UBSan must pass;
no mocks replace native Core handles. Existing C/compiler-derived ABI layout
tests remain unchanged. Source review composes the already qualified PnP and
AP3P reference/refinement numerical contracts. No C++ implementation, native
algorithm, dependency or ABI change; private ABI remains 48. Warnings stay errors.

Hosted gates: Linux, macOS, repository-checks, Linux sanitizers, and manual pinned
OpenCV 4.1.0/calib3d, 4.10.0/calib3d, 5.0.0/geometry matrix. The new example is
included in Linux/macOS and pinned-version jobs. CI trigger/job topology and
Windows post-merge policy are unchanged. All claimed final passes must match the
exact final pushed PR head. Keep PR open, non-draft, unmerged, auto-merge disabled;
no amend, force-push, tag, release or version bump. Stop at independent review.

## Limitations

Fitting and held-out errors can share systematic bias. Passing caller policy
establishes compatibility only with supplied landmarks/observations/calibration,
not independent physical truth, global correctness, covariance or uncertainty.
Native EPNP sampling and Rodrigues representations can vary by OpenCV version;
tests use actual consensus and physical geometry rather than fixed sample
sequences or fixed noisy consensus counts. Diagnostics after refinement do not
reclassify native consensus identities. Exact overlap checking is quadratic and
has no invented near-duplicate threshold. Use a suitable local Cartesian frame.