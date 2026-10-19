# Held-out known-plane / empirical homography consistency (Task 028)

`Verify_Known_Plane_Homography_Consistency` compares two independently sourced
algebraic models without adjusting either supplied pose or the supplied plane.

```text
Supplied world-to-camera poses + plane --> Task 027 predicted normalized H12
Fitting raw Brown pixels (two cameras) --> separate batch undistortion
                                      --> normalized pairs --> one robust H fit
Held-out raw Brown pixels (two cameras) --> separate batch undistortion
                                      --> predicted and empirical transfers
                                      --> three independent normalized distances
```

## Domains, frames and independent evidence

1. Inputs are raw Brown-distorted pixels, with independent zero-skew intrinsics
   and five Brown coefficients for each camera.
2. Ideal undistorted pinhole pixels retain focal/principal-point scaling. No
   ideal-pixel matrix is used here, and it must not map arbitrary distorted pixels.
3. All fitted/transferred coordinates and comparison limits are dimensionless
   normalized camera coordinates: **not pixels**. `Undistort_To_Normalized` is
   called in whole-array batches for each camera and each data split.

Normalized fitting points are explicitly converted to `Image_Point_Array` only
for structural compatibility with the existing estimator. Its legacy field
`Reprojection_Threshold_Pixels` receives `RANSAC_Threshold_Normalized`; the actual
domain is normalized throughout. Native homography estimation still has Float32
fitting precision limitations. Iterative Brown inversion is approximate, with no
universal distortion or RANSAC accuracy guarantee.

The pose convention remains `X_camera = R * X_world + t`; `t` is not camera
location and `C_world = -R^T*t`. Both poses and the infinite plane share one
Cartesian world frame and length unit. Task 027 constructs first-to-second H
using Camera-2-frame relative translation. No geometric fallback or pose/plane
optimization is performed.

The four input arrays are paired positionally within each split, never combined.
Fitting requires at least five pairs (the robust-only minimum); held-out requires
at least one. Held-out observations never enter `Estimate_Homography_RANSAC`.
Different counts and lower bounds between splits are supported, including
multi-element arrays ending at `Positive'Last`. Output indices are one-based
positions, not caller subscripts. Feature identity, sampling separation, and
statistical independence of the withheld data are caller responsibilities;
coordinates cannot establish them. Even distinct coordinates need not be
statistically independent.

## Complete validation and batched control flow

Before any native call validate all four arrays/counts, every finite pixel,
both intrinsics (finite positive focal lengths), all ten distortion coefficients,
both complete poses, finite plane anchor and finite nonzero normal, iteration
bound, finite positive Float32-representable RANSAC threshold, confidence strictly
between zero and one, minimum support in `4..fitting_count`, and all three finite
nonnegative held-out limits. There are no default numeric policy fields.
Invalid late held-out coordinates or policies cannot consume a native checkpoint.

After validation, construct the predicted model exactly once. If undefined,
return `Predicted_Model_Unavailable`, retaining the complete Task 027 outcome,
held-out count/positions and guarded unavailable diagnostics; no empirical fit
is attempted. This includes first/second-center planes and numerical absence.
Exact common-center pure rotation, including a plane through the common center,
may participate algebraically but is not proof of a physical plane intersection.

For defined geometry: undistort both fitting batches, fit exactly once, undistort
both held-out batches, transfer the entire predicted held-out batch once using
`Transfer_Normalized_Plane_Points`, then use `Map_With_Homography` for empirical
points. There is no per-point native operation, duplicate projective arithmetic,
solver or estimator. Even weak support/no-model preserves independently evaluable
predicted held-out diagnostics. `Found=False` is a legitimate successful native
no-model outcome, not an exception or a verified physical model.

## Owned results and availability

The private aggregate is a discriminated, ordinary owned Ada value. The controlled
empirical estimate remains local; only the final matrix and final one-based fitting
inlier positions are copied. No native pointer or limited object escapes. Final
inlier identities retain the existing independent final-H classification, in
original order, valid, unique and strictly ascending. Their count is zero when
no empirical model was found. `Empirical_Homography` requires `Empirical_Found`.
`Predicted_Model` always returns the complete Task 027 result with its own guards.

`Holdouts` returns complete owned diagnostics. Each contains `Original_Position`,
independent target availability flags, guarded normalized targets, independent
`Metric_Available` flags, guarded `Normalized_Error`, `Disposition`, and `Accepted`.
The metrics are:

* `Predicted_Observed`: Euclidean distance from predicted target to normalized B.
* `Empirical_Observed`: Euclidean distance from empirical target to normalized B.
* `Model_Separation`: Euclidean distance between the two transferred targets.

All available metrics are computed before testing any limit. Observation rejection
never suppresses available model separation. An unavailable mapping or error is
not `(0,0)` or zero evidence. Accessing absent values raises `OpenCV_Error`.
Checked subtraction and a max-component-scaled Euclidean norm retain exact zero,
tiny nonzero distances and large finite components without naive squaring.
Nonfinite differences, overflowing magnitude, zero projective denominator or
unrepresentable target coordinates are independently unavailable. Finite Float64
rounding/cancellation and conservative projective arithmetic limitations remain.

Per-point precedence is exactly enum order:

1. `Predicted_Transfer_Unavailable`.
2. `Empirical_Transfer_Unavailable`.
3. `Holdout_Arithmetic_Unavailable` if any required metric is unavailable.
4. `Predicted_Observation_Rejected`.
5. `Empirical_Observation_Rejected`.
6. `Model_Agreement_Rejected`.
7. `Holdout_Consistency_Accepted`.

The first applicable state wins; other independently evaluable diagnostics remain.
All three comparisons are **inclusive <=** their explicit limits, without hidden
epsilons, clamping, depth, field-of-view, or physical range checks.

Aggregate precedence is predicted unavailable, empirical not found, insufficient
final fitting support, any held-out rejection, then accepted. Acceptance requires
defined prediction, found empirical model, sufficient caller-selected final
support, and **every** held-out point evaluable and passing **all three** limits.
One failure rejects the aggregate without erasing neighboring results.

## Failure boundary and limits of interpretation

Malformed requests and native failures raise `OpenCV_Error` with no partial return;
failed assignment preserves an earlier complete result. Native estimation and its
accessors, construction and undistortion are outside residual arithmetic handlers.
Only a narrowly scoped Ada subtraction/norm helper converts arithmetic failures
to per-metric unavailability. Resource ownership remains in reused Core APIs and
the local controlled estimate. The private C ABI remains **48 operations**; no
production C++, dependency, or test hook is added.

Low normalized error does not prove pose accuracy, feature identity, a correct
plane, or independent sampling. A corrupted fit can be internally consistent and
wrong for the scene. RANSAC confidence is an algorithmic control, not a posterior
probability of navigation correctness. Agreement between H models and agreement
with held-out observations are different tests; both are required here.
Tasks 024–026 separately establish their physical forward ray-plane, positive-depth,
range, distorted-pixel and reciprocal world-separation policies. Algebraic
acceptance here is not a substitute for those contracts.

No geodesy, DEM/DTED lookup, image warp, feature matcher, SLAM, bundle adjustment,
uncertainty propagation or navigation fusion is added. The sixteenth example is
entirely synthetic and makes no physical navigation-accuracy claim. See
[execution evidence and developer review](plane-homography-consistency-validation.md).