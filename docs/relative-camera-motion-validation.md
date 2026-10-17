# Relative camera motion validation

The test oracles construct rotation matrices and camera centers independently of the production path. Production inversion publishes through native Rodrigues and native `composeRT`. Tests compare matrices and transformed points, not Rodrigues coefficient arrays.

Covered cases:

- Identity inversion and identical-camera relative motion, including identical nonidentity poses, exact zero displacement, and an undefined direction.
- Translation-only analytic centers, relative translation `t2 - t1`, and baseline `sqrt(74)`.
- Noncommuting `Rz(pi/3)` and `Rx(pi/4)`, including reversed order and agreement with `Compose_Poses`.
- Pure rotation at a common origin has exact zero baseline. At a constructed nonzero common center, native rounding can leave a sub-1e-15 residue; that residue is not redefined as zero or required to occur on every backend.
- General nonidentity rotations and translations checked with an independent closed-form axis-angle matrix oracle, point composition, and direction composition.
- Double inversion, inverse relative motion, and swapped-camera displacement sign with unchanged baseline length.
- Native `projectPoints` with zero and nonzero Brown distortion.
- A genuinely tiny baseline against identity (`sqrt(21) * 1e-300`), a large known 3-4-5 length (`5e300`), and corresponding unit directions. `1e15 + 3` remains representable; larger magnitudes can absorb the increment before subtraction.
- Nonfinite components, unrepresentable rotations or center differences, and finite displacement components whose norm overflows.
- Frame-order and incorrect-sign regressions. Relative translation is not copied from world displacement.

Local qualification is recorded only after the commands below are executed for the final commit. Do not treat this document as evidence that a later edit passed.

```text
alr -n build
alr -n -C tests build
alr -n -C tests exec -- sh ../scripts/run_native.sh bin/run_tests
alr test
alr -n -C examples build
alr -n -C examples exec -- sh ../scripts/run_native.sh bin/relative_motion_synthetic
python3 scripts/check_repository.py
python3 -m unittest discover -s tests/configuration -v
sh scripts/run_profile_tests.sh
alr -n exec -- sh scripts/run_sanitizers.sh native
alr -n -C tests exec -- sh ../scripts/run_ada_faults.sh
alr -n exec -- sh scripts/run_sanitizers.sh
```

The private C ABI remains 48 operations. No new native source is added.

## Local qualification captured

On Linux with GNAT 16.1.0, Alire 2.1.1, and OpenCV 4.10.0:

- Production build and `alr -n test`: passed, including 163/163 AUnit tests,
  compiler-derived layout checks, production/fault-injection actual-shim native
  boundaries, and Ada fault translation/cleanup tests.
- All six synthetic examples: passed. The final relative-motion example also
  checks native first-camera projection against an analytic Rz transform and
  scalar Brown projection; maximum transform/projection difference was
  `2.27373675443232e-13`.
- C header and all source/profile tests: passed.
- Actual-shim ASan/UBSan, production and fault-injection variants: passed.
- Repository checks, all 26 configuration tests, shell syntax checks, and
  `git diff --check`: passed.
- After adding the isolated world-displacement-overflow fixture, the AUnit
  build and all 163 tests were rerun successfully.

Logs were captured under `/tmp/calib3d-018-qualification/` in the qualification
workspace. Hosted cross-platform and pinned-version results must be checked
against the PR head SHA; local results do not establish those gates.
