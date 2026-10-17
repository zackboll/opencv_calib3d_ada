# AP3P selection qualification

Local OpenCV 4.10.0 / calib3d on Linux. This is not hosted CI evidence and
not the pinned 4.1.0 / 4.10.0 / 5.0.0 matrix. Windows was not run; its
workflow file was not changed.

## Executed locally

- `alr -n build` succeeded.
- `sh scripts/test.sh` exited 0. AUnit: 140 run, 140 passed, 0 failed
  assertions, 0 unexpected errors. This includes the owned-pose provenance
  case. The script also ran the raw production and fault-injection boundaries
  and Linux ASan/UBSan on the actual shim.
- `scripts/run_ada_faults.sh` passed, including
  `AP3P selection propagates native projection faults without a result`.
- `python3 scripts/check_repository.py` passed: 48 ABI imports/exports and
  139 registrations.
- `python3 -m unittest discover -s tests/configuration -v`: 26 passed.
- `scripts/run_profile_tests.sh` and `sh -n scripts/*.sh` passed.
- All five examples ran. `pnp_synthetic` printed
  `UNIQUE_SELECTED_HYPOTHESIS` for the one-pixel policy and
  `AMBIGUOUS_QUALIFIED_HYPOTHESES` for a 1000-pixel separation. The observed
  separation was `1.46776468841858E+02` pixels. Candidate index 1 on this
  OpenCV build is not a cross-version contract.

The 129 existing registrations remain. The eleven new cases cover known
fourth-reference geometry, five nonzero Brown coefficients, insufficient and
exact score separation, no hypotheses, no references, rejected candidates,
behind-camera depth, numerical unavailability, invalid thresholds, differing
lower bounds, order invariance, `Positive'Last`, and owned-pose provenance.
The owned pose is returned by `Selected_Pose` with no second solver call.

## Oracles

The known pose uses an independent `Rz(0.22) * Ry(0.35)` matrix and
translation `(0.2, -0.3, 0.8)`. Tests compare rotation by the trace angle,
not Rodrigues coefficients. Camera center is checked against `-R^T * t`.
The distorted fixture uses a test-only Brown equation; production Ada does
not contain that polynomial and checks native `Project_Points`.

On this build the known fourth reference selected RMS
`2.84217094304040E-13`. Five distorted references selected RMS
`1.56267682819610E-04` and maximum `2.01903939938354E-04`, inside the
caller's `1.0E-3` policy. Reversing candidate order preserved the selected
rotation and translation.

## Limitations

A unique result depends on the supplied references and the caller's finite
nonnegative thresholds. It is not global localization, a covariance, or a
geographic position. Native AP3P order remains version-specific. No new C
ABI operation was added.
