# Task 019 validation ledger

## Baseline

Started from clean fetched `origin/main`:
`fcd3ca5ca9030444ce2d95dd15d6ce92aacdc9d7`, the Task 018 PR #19 merge.
Branch: `feature/019-metric-world-triangulation`.
Baseline actual execution: 163/163 AUnit tests; 48 private ABI operations.
Task 018 post-merge runs completed successfully (not pending):

* [Cross-platform, all four jobs](https://github.com/zackboll/opencv_calib3d_ada/actions/runs/39061754390).
* [Windows post-merge](https://github.com/zackboll/opencv_calib3d_ada/actions/runs/39061754341).

## Independent numerical oracles

The rectified baseline 2 fixture uses analytic observations `(0.5,0.125)` and
`(0.25,0.125)`, expecting world `(4,1,8)` and both depths 8. Qualified absolute
component/depth tolerance is `1e-12` for this small, well-conditioned fixture.

The general fixture uses analytic noncommuting Rz(0.3) and Rx(-0.2), nonzero
translations, unequal focal lengths/principal points, and separate all-five nonzero
Brown polynomials. Test-only scalar projection is independent of production
normalization/triangulation. Native projection must agree within `1e-10` pixels.
Both original and reversed cameras must recover world `(1.4,0.6,8)` within `1e-9`
in this moderate-distortion fixture. These fixture tolerances are not universal
API accuracy bounds; iterative undistortion and native SVD rounding vary.

Local example on OpenCV 4.10.0 reports:

* Baseline: `2.00888231075949` caller length units.
* Maximum reconstructed world component error: `2.30926389122033e-14`.
* First/second pixel errors: `5.79691403981176e-13`, `8.03887338846093e-14`.
* Acute parallax: `0.237017311462654` radians.

Tiny baseline `2e-300` and large baseline `2e300` reconstruct the same unit-gauge
scene at their proper scales. The large-origin fixture uses a representable
2-unit baseline at origin magnitude `1e12`; its `0.0002` X tolerance reflects
Float64 spacing at that origin, not a general world-accuracy policy.

## Coverage and execution

170 registrations preserve the original 163 and add seven grouped metric cases.
Local qualification executed 170/170, zero failed assertions/unexpected errors.
Production build, all seven examples, native boundary and fault campaigns, Ada
cleanup/failure tests, actual production/fault shim ASan+UBSan, C++ profile tests,
26 Python configuration tests, repository checks, shell syntax, and diff checks
passed. Final-head hosted execution is a separate gate reported on the PR.
Existing fault checkpoints 1/2, 13/14, 15/16, 37..41 exercise 55 native exceptions
through the public operation. Existing homogeneous controls execute real native
triangulation first, then test source undefined/unrepresentable statuses, metric
depth underflow, large finite unit coordinates, unrepresentable world transform,
and pixel residual overflow. Controls are not claims about unmodified native
solver output. Zero baseline must preserve an armed triangulation checkpoint.

No C++ source, native hook, public native handle, dependency, crate version, or
private ABI operation was added. ABI remains 48. Windows workflow is unchanged;
Task 019 Windows qualification is after merge only.

Full local and exact-final-head hosted qualification results are recorded on the
review PR. Do not infer completed hosted qualification from this test inventory.
Local logs: `/tmp/calib3d-019-qualification/` (Linux, GNAT 16.1.0, Alire 2.1.1,
OpenCV 4.10.0/calib3d).

## Commands

Run Alire mutation commands serially:

```sh
sh scripts/test.sh
alr -n -C examples build
# Run all seven bin/*_synthetic executables via scripts/run_native.sh.
alr -n exec -- sh scripts/run_sanitizers.sh
python3 scripts/check_repository.py
python3 -m unittest discover -s tests/configuration -v
sh scripts/run_profile_tests.sh
for script in scripts/*.sh; do sh -n "$script"; done
git diff --check
```

The review gate requires actual Linux/macOS/repository/sanitizer execution and
manual OpenCV 4.1.0, 4.10.0, 5.0.0 matrix execution on the exact final pushed head.
Leave the PR open, non-draft, unmerged, with auto-merge disabled.
