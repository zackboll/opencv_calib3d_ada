# Task 020 qualification ledger

## Baseline

Clean fetched `origin/main` was
`19e6a9d4da8ca2d449b3f6cf11ef007d27105bfc`, Task 019 / PR #20 merged.
Feature branch: `feature/020-metric-point-refinement`.
Baseline local `scripts/test.sh` executed 170/170 AUnit tests successfully,
including existing native boundary/fault and Ada fault campaigns. Repository
checks verified 170 registrations and 48 private C ABI declarations/imports.
All current AGENTS and `.clinerules/` instructions and the five prerequisite
geometry/refinement contracts were reviewed before implementation.

Main runs at that exact SHA:

* [Cross-platform](https://github.com/zackboll/opencv_calib3d_ada/actions/runs/39082305969): completed, success.
* [Windows post-merge](https://github.com/zackboll/opencv_calib3d_ada/actions/runs/39082306057): initially in progress; subsequently completed, success.

## Independent numerical oracles

177 registrations retain all 170 originals and add seven cohesive grouped cases.
The noisy rectified scene has B=2, observations (.5,.126), (.25,.124).
Independently, X/Z=.5, (X-2)/Z=.25, Y/Z=mean(.126,.124), giving
world (4,1,8), both depths 8, both normalized errors .001.
World tolerance is 1e-8 in this small well-conditioned fixture, not a universal
bound. Original full records are compared exactly against a separate initial
reconstruction. Measured positive pixel worsening is used as an exact inclusive
allowance boundary; zero allowance rejects it despite normalized improvement.
A second-camera pixel threshold strictly between initial and refined errors
demonstrates rescue of an initially policy-rejected but usable seed.

Noncommuting Rz(.3)/Rx(-.2), nonzero translations, unequal intrinsics, and two
independent all-five Brown polynomials use scalar analytic forward projection,
not production transforms to generate expectations. Added noise is .2 pixels
in first y and -.15 pixels in second x. Original/reversed results are checked
against known world (1.4,.6,8) within .02 caller units and against each other
within 1e-7. These explicitly qualified noisy-fixture tolerances accommodate
native SVD/undistortion variation; Rodrigues components are never compared.

Grouped cases cover tiny/large baselines (2e-300, 2e300), origin 1e12 with
representable 2-unit displacement, initial conversion overflow, zero baseline,
infinite/behind-camera sources, low parallax, rejected/unavailable mixed batches,
extreme differing lower bounds including `Positive'Last`, empty requests, and
invalid counts/calibration/distortion/poses/options/allowances including NaN/Inf.
The exact native noiseless SVD seed may or may not admit a Float64 improvement;
tests permit either while checking physical geometry, rather than asserting a
version-specific intermediate outcome.

Clearly labeled synthetic post-native homogeneous controls separately establish
zero-objective/no-step preservation, optimizer numerical unavailability, improved
metric overflow/underflow rejection,
and mixed unavailable/accepted batch independence. They run native triangulation
first and then replace its first seed; they are not claims about unmodified SVD
output. Existing native checkpoints exercise 55 exceptions through the new
public operation with assignment/batch atomicity checks. No C++ hook was added.

The eighth example is `metric_refinement_synthetic`. Its independent analytic
truth is (4,1,8); unequal intrinsics and inconsistent y noise yield a genuinely
improved seed. It prints unit and metric/world coordinates, both normalized RMS
values, both initial/final pixel errors, disposition, and known-truth error,
and exits unsuccessfully on failed objective/acceptance/geometric assertions.

## Execution policy

Local logs: `/tmp/calib3d-020-qualification/` (Linux, Alire 2.1.1,
GNAT 16.1.0, OpenCV 4.10.0/calib3d). Inventory is not a claim of execution.
Local qualification completed: production build; 177/177 AUnit tests, zero
failed assertions or unexpected errors; all eight examples; production and
fault-injection native boundary drivers; all Ada fault campaigns including the
new orchestration; actual-shim production and fault-injection ASan/UBSan;
C++ profile tests; 26/26 Python configuration tests; repository checks; shell
syntax; and `git diff --check` passed. No C++ source, private ABI, dependency,
crate version or Windows workflow change was made.
Exact final-head execution results and hosted URLs are recorded on the review
PR. All success claims must match its final pushed head.

Run Alire commands serially:

```sh
sh scripts/test.sh
alr -n -C examples build
# Run all eight bin/*_synthetic using scripts/run_native.sh.
alr -n exec -- sh scripts/run_sanitizers.sh
python3 scripts/check_repository.py
python3 -m unittest discover -s tests/configuration -v
sh scripts/run_profile_tests.sh
for script in scripts/*.sh; do sh -n "$script"; done
git diff --check
```

Hosted gates are Linux, macOS, repository-checks, Linux sanitizers, and manually
dispatched OpenCV 4.1.0/calib3d, 4.10.0/calib3d, 5.0.0/geometry matrix. Windows'
post-merge workflow and all trigger policies are unchanged. No C++/ABI changes,
version bumps, releases, tags, merging, amending, or force pushes are permitted.
Stop at independent review with the PR open, non-draft, auto-merge disabled.
