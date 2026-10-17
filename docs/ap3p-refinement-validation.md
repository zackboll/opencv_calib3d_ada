# AP3P refinement qualification

Local OpenCV 4.10.0 / calib3d on Linux. This file records local execution
only. It is not hosted CI evidence and not the pinned 4.1.0 / 4.10.0 / 5.0.0
matrix. Windows was not run; its workflow file was not changed.

## Executed locally

- `alr -n build` succeeded.
- `alr -n exec -- sh scripts/run_sanitizers.sh` exited 0. It printed
  `PASS: production ASan+UBSan actual-shim boundary` and
  `PASS: fault-injection ASan+UBSan actual-shim boundary`, with leak detection
  and halt-on-error enabled. No ASan or UBSan diagnostic was printed.
- `alr -n -C tests exec -- sh ../scripts/run_native.sh bin/run_tests` exited 0.
  AUnit: 153 run, 153 passed, 0 failed assertions, 0 unexpected errors.
  This preserves the previous 140 registrations and adds 13 refinement cases.
- `alr -n -C tests exec -- sh ../scripts/run_ada_faults.sh` passed, including
  `synthetic AP3P refinement false and projection/refinement faults publish no partial result`.
  The false result is a test-only fault injection. It is not a naturally
  observed native failure.
- `alr -n -C examples exec -- sh ../scripts/run_native.sh bin/pnp_synthetic`
  exited 0. It printed `UNIQUE_SELECTED_HYPOTHESIS`, refinement disposition
  `ACCEPTED`, held-out RMS before `6.03130386554869E-01`, and held-out RMS
  after `2.96679898276507E-01`.

The new cases cover a known nonidentity pose, deterministic fitting noise,
all five nonzero Brown coefficients, held-out worsening, a caller maximum
rejection, nonpositive held-out depth, numerical unavailability, no unique
selection, duplicate world points, invalid thresholds and lengths, different
lower bounds, `Positive'Last`, and candidate-order independence. No test
requires a fixed native AP3P candidate index.

## Oracles

The known pose uses an independent `Rz(0.22) * Ry(0.35)` matrix and
translation `(0.2, -0.3, 0.8)`. Tests compare rotation by the trace angle,
not Rodrigues coefficients. Camera center is checked against `-R^T * t`.
The distorted fixture uses a test-only Brown equation. Production Ada does
not contain that polynomial; it checks native `Project_Points`.

On this build the shifted-selection worsening case moved held-out RMS from
`3.09609705830254E-13` to `7.59637993684885E+00` and returned
`REJECTED_BY_POLICY`. The accepted clean and distorted poses matched the
geometric rotation, translation, and camera center within the stated test
tolerances.

## Limitations

Acceptance depends on the caller's finite nonnegative thresholds and on the
held-out observations. It is not global localization, a covariance, or a
geographic position. Disjoint correspondence sets do not remove shared
systematic error. Native AP3P order remains version-specific. No new C ABI
operation was added.
