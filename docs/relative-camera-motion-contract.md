# Relative camera motion contract

Task 018 converts two absolute world-to-camera poses into a relative camera-to-camera transform. Both inputs must already use the same caller-defined Cartesian world frame and the same length units. These operations do not estimate a pose and do not decide whether the inputs are accurate or share a real-world reference. The caller remains responsible for that relationship.

No new C ABI operation is added. The private ABI remains 48 operations. Inversion and relative motion reuse native Rodrigues, native camera-center, and native `cv::composeRT`.

## Frames

An existing pose maps a source frame to a destination frame:

```text
X_destination = R * X_source + t
```

`World_To_Camera_Pose` is reused as that rigid record even when the source and destination frames are reversed. `t` is never a camera position.

`Invert_Pose` maps destination coordinates back to source coordinates:

```text
R_inverse = R^T
t_inverse = -R^T * t
```

The published rotation is native Rodrigues output. Compare `Rotation_Matrix_Of`, not Rodrigues components. A negated Rodrigues vector is accepted only when native Rodrigues of that vector reproduces `R^T`.

`Camera_To_Camera_Pose (First, Second)` maps coordinates in the first camera to coordinates in the second camera:

```text
X_first  = R1 * X_world + t1
X_second = R2 * X_world + t2
R_relative = R2 * R1^T
t_relative = t2 - R_relative * t1
X_second = R_relative * X_first + t_relative
```

The implementation is `Compose_Poses (Invert_Pose (First), Second)`. Order is noncommutative. `t_relative` is expressed in the second camera frame. It is not the second camera's position and it is not the world-frame displacement.

## Baseline diagnostics

`Measure_Camera_Motion` returns owned Ada values:

- relative transform from the first camera to the second camera;
- both camera centers in world coordinates, `C = -R^T * t`;
- the second center expressed in the first camera frame, `R1 * (C2 - C1)`;
- world displacement `C2 - C1`;
- baseline length, the scaled Euclidean norm of that displacement.

A baseline direction is defined only when the displacement is not the exact zero vector. Exact zero displacement, including identical poses, leaves the direction undefined. A numerically nonzero residue is not treated as zero. There is no epsilon that collapses a nonzero length to zero.

Baseline units are the caller's world length units. Relative scale exists only because both absolute poses use that same Cartesian frame. No Earth, geographic, or navigation uncertainty is implied.

Nonfinite input and unrepresentable intermediate or final arithmetic raise `OpenCV_Error`. The operations never return NaN, infinity, a saturated coordinate, or a fabricated zero displacement.

The operations are not coupled to `Solve_PnP_RANSAC`, `Select_AP3P_Pose`, or `Select_Refine_AP3P_Pose`. Any validated `World_To_Camera_Pose` may be compared.
