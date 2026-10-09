# Task 010 validation

The independent AUnit oracle constructs `K R K^-1` for Rz(0.12), and
`K (Ry(0.08)+[-0.12,0.04,0.08]*[0,0,1]^T) K^-1` with
`K=[800,0,640;0,820,360;0,0,1]`. It checks all candidates rather than assuming
ordering. It covers known first-to-second motion, zero pure vectors, native
counts one/four, translation magnitude sqrt(0.0224), and simultaneous sign
pairs. Each candidate reconstructs normalized H up to independently fitted
common scale for H, 7H, -7H, 1e-6H, and 1e6H.

Local OpenCV 4.10 results: maximum reconstruction error 3.96e-16; all 83 AUnit
registrations executed/passed, zero failed assertions/unexpected errors.
Compiler-derived candidate layout is 120 bytes/alignment 8; aggregate 488/8,
count offset 0, candidate offset 8, stride 120. All fifteen offsets and all
four C-written/Ada-read records are checked, not assumed from those numbers.

Raw production/fault checks cover proper candidate rotations, null/zero/
singular/nonfinite matrices and invalid intrinsics. Ada covers unrepresentable
canonicalization too. Exception checkpoints 42..45 exercise all five categories
in both raw C and Ada. Post-real-call controls explicitly simulate zero count,
malformed shape/depth, invalid count, and nonfinite output. Failure aggregates
must have count and every candidate field zero. Controls do not exist in
production. The existing macOS-safe fault isolation is retained.

The homography example preserves robust estimation and adds separate calibrated
pure/general demonstrations with matrices, every hypothesis, and projective
reconstruction errors. No unique physical or metric solution is claimed.

## Executed local serial campaign

On local OpenCV 4.10.0 / calib3d, production root/tests/examples builds passed
with warnings treated as errors. All 83 AUnit cases executed/passed with zero
assertion failures/unexpected errors. `alr test` passed. The 24 Python
configuration/repository tests, repository static checks (44 exact imports/
exports), profile helpers, shell syntax checks, and `git diff --check` passed.
Actual-shim production and fault-injection ASan/UBSan runs passed with leak
detection and halt-on-error enabled. All five examples passed:
`pnp_synthetic`, `homography_synthetic`, `fundamental_synthetic`,
`essential_synthetic`, and `triangulation_synthetic`. Homography example
reconstruction maximum was 3.96e-16, with one pure and four general hypotheses.

The first example build caught warnings from cross-root Ada array conversions;
explicit coefficient copies corrected these before the complete serial rerun.
No warning policy was weakened.

Exact final-head ordinary/manual CI evidence will be reported at the review
gate; source review and local 4.10 execution alone are not 4.1/5.0 execution
evidence. Task 009 post-merge Windows run 37872001005 passed on
`8a4a09c573ccb2086650a644a6e688ed8fa577b5`; Task 010 Windows qualification
remains post-merge only.