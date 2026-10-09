# Task 009 qualification

Starting main: `4e640a1c78c0230a3ab8b27def132558a5153df2`.
Branch: `feature/009-normalized-point-refinement`.
Version remains `0.1.0-dev`; Core pin remains
`4da9d35ea21e1b2efe96296243ea668b488c6326`.

Task 008 post-merge runs both completed successfully:
- Windows `37716673803`: 74 executed/passed, zero assertions/errors;
  DLL and import-library inspection passed (native 5.0 geometry).
- Cross-platform `37716673912`: repository-checks, linux, macos and
  linux-sanitizers passed; Linux/macOS each executed/passed 74 cases.

## Numerical evidence (local OpenCV 4.10.0 / calib3d)

80 AUnit cases registered/executed/passed, zero failed assertions/unexpected
errors in the development run. Six added registrations cover independent
Jacobian, scaled damped solve/sign, exact/noisy fixed-pose geometry and bounded
prefix monotonicity, mixed/parallax preservation, malformed arguments, and
extreme finite coordinates/depths. Nonfinite fixtures locally suppress validity
checks only to inject deliberately invalid caller values, matching existing tests.

Central finite differences independently project rectified and nonidentity,
non-axis-aligned poses, checking all 12 components per pose at `1e-9` absolute
tolerance with step `1e-5`. Maximum observed error: `3.25066362716342e-12`.
The hand-controlled diagonal-J solve checks the scaled normal residual and sign.
Nonpositive-depth and overflowing projection trials are rejected.

| Fixture | Outcome | Steps | Independent initial RMS | Independent final RMS |
|---|---|---:|---:|---:|
| Exact `(2,1,5)` rectified | Improved | 1 | 6.93889390390723e-17 | 0 |
| Rectified noisy | Improved | 3 | 1.41435669011419e-2 | 1.41421356237310e-2 |
| Nonidentity noisy | Improved | 5 | 1.34472706224152e-2 | 1.34455731780163e-2 |

Noisy observations perturb the second projection by `(+0.025,-0.04)`;
seeds use real native DLT triangulation. Before/after objectives are independently
computed from projection equations, not just reported RMS. Exact final XYZ is
not compared across versions. Prefix budgets 1..20 check the specified robust
norm's strict accepted-step monotonicity, avoiding a differently rounded squared
sum as an oracle for last-bit comparisons.

Low-parallax fixture: screened `Skipped_Low_Parallax` at `1e-6`; default zero
attempts refinement (locally Improved). Non-Usable mixed entries preserve all four
statuses exactly. Extreme finite cases include unrepresentable initial projections,
subnormal positive depth, and maximum finite depth; no coordinate clamps or
escaping Constraint_Error. Safe improvement is allowed even at extreme scale.

40-point Essential composition: 40 selected/triangulated/refined; 35 Improved,
5 No_Improving_Step, zero other outcomes in the local development run. Positive
updated depths, finite errors, nonworsening evaluated RMS and immutable initial
array are checked. Clean geometry need not take updates.

## Qualification status

Full serial local qualification passed: repository checker (43 ABI / 80 AUnit),
24 Python configuration tests, six C/C++ profile helpers, seven shell syntax
checks, library/test/example builds, direct AUnit runner and `alr test` (80/80
each, zero assertions/errors), raw production/fault variants, combined ASan+UBSan
production/fault variants with leak detection and halt-on-error, and all five
examples. `git diff --check` passed. The final additional coupled SPD oracle was
built/run with 80/80 passed; its independently evaluated normal-system residual
is bounded by `1e-13`. The exact manually supplied zero-objective case retains
the point exactly despite deliberately stale finite reported diagnostics.

Logs are retained locally under `/home/zboll/task009-qualification/`. A transient
full `/tmp` tmpfs prevented creation of a follow-up log; no test defect was
reported. Follow-up logging moved to the home filesystem and passed.
Remote final-head CI/matrix evidence is recorded in the PR after execution;
local inventory is not a claim about unrun targets.
The three native ABI source files are unchanged; private ABI remains 43.
Existing actual-shim native sanitizers regress C++ production/fault variants;
the new pure-Ada numerical solver is not directly ASan-instrumented.