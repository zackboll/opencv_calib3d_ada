# Pose composition contract

`Compose_Poses (First, Second)` binds native `cv::composeRT` through one guarded
private C operation, `opencv_calib3d_compose_rt`. Ada validates, marshals,
translates exceptions and validates the result. Ada does not compose poses.

## Frame meaning

```text
First:   X_B = R1 * X_A + t1        (frame A -> frame B)
Second:  X_C = R2 * X_B + t2        (frame B -> frame C)
Result:  X_C = R3 * X_A + t3        (frame A -> frame C)
R3 = R2 * R1        t3 = R2 * t1 + t2
```

"Apply `First`, then apply `Second`." Operands must not be swapped; the
operation is not commutative. The caller supplies consecutive compatible
frames; no frame identifier exists at run time. Both translations must use the
same Cartesian unit. No geographic frame, navigation position or accuracy is
implied. These equations are used only in test oracles.

## Rodrigues representation

Rotations are Rodrigues vectors. Equivalent physical rotations have different
vectors, and the native result is returned unchanged (no Ada canonicalization or
normalization). Compare `Rotation_Matrix_Of`, never vector components.

## Validation

* Ada rejects any nonfinite component of either input before native entry.
* The shim also rejects null pointers and finite rotation vectors whose
  squared component sum overflows Float64 (see source review for why).
* The native result must be 3x1 Float64, finite, and a proper rotation
  (orthonormality and determinant within 1e-12).
* A composed translation that overflows is a native nonfinite result and is
  rejected. Nothing is clamped, wrapped or saturated.

## Failure atomicity and aliasing

For a nonaliased call the result is cleared first, a complete local pose is
staged, and it is published once on success. On any failure the output stays
all zero and no C++ exception crosses the ABI.

`result == first` or `result == second` is rejected **before** clearing, so an
input pose is never destroyed by a validation failure. This is an early
rejection distinct from ordinary output clearing. Ada always passes three
separate records.

## Private ABI

The existing six-double `opencv_calib3d_pose` is reused. The private ABI grows
from 46 to 47 operations. No handle, generator or Jacobian output exists.
Fault checkpoints 54 (before native), 55 (after native), 56 (before output
validation) and 57 (before publication) are new; 53 was the previous maximum.

## Known native behavior

A composed rotation with physical angle below roughly 1e-5 is returned as an
exact zero vector (matrix-to-vector Rodrigues flush). This is a native
behavior, not a binding choice; the associated matrix error is below 1e-8.
