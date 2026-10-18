# Known-pose calibrated two-view epipolar verification (Task 022)

```text
Two known world-to-camera poses
        |
 Relative camera-to-camera pose
        |
 E = [t_hat]_x R
        |
 Independently calibrated image correspondences
        |
 Normalized Sampson errors
        |
 Whole-set caller-controlled verification
```

This is geometric consistency verification. No pose or Essential estimation,
RANSAC, refinement, SVD, rotation repair, reconstruction, or matching is performed.
Neither supplied pose is changed. All code added to production is handwritten
Ada. The C++ implementation and 48-operation private C/Ada ABI are unchanged.

## Frames and Essential construction

Both poses must use the same caller-defined Cartesian world frame and length
units. The caller is responsible for that relationship and for pose accuracy.

```text
X_first  = R1 X_world + t1
X_second = R2 X_world + t2
R_relative = R2 R1^T
t_relative = t2 - R_relative t1
X_second = R_relative X_first + t_relative

C1 = -R1^T t1; C2 = -R2^T t2
displacement_world = C2 - C1
physical_baseline = norm(displacement_world)
t_hat = normalize(t_relative)
E = [t_hat]_x R_relative
[t_hat]_x = [[0,-tz,ty],[tz,0,-tx],[-ty,tx,0]]
x2^T E x1 = 0; x_i = (normalized_x_i,normalized_y_i,1)
```

World-to-camera `t` is **not** camera position. World camera-center displacement
is not the relative translation term; the latter is expressed in the **second**
camera frame. `Translation_Direction` normalizes this signed term, not the world
displacement. `Baseline_Distance` is the physical magnitude in caller world units.
It does not multiply E: an Essential matrix has arbitrary nonzero common scale.
Reversing both cameras gives E transpose, within qualified floating-point error.

`Essential_From_Camera_Poses` calls `Measure_Camera_Motion`, which already uses
`Camera_To_Camera_Pose`. It does not independently duplicate pose composition.
The existing scaled `Unit_Vector` helper divides by the maximum absolute component
before squaring. Ada builds the nine coefficients with checked finite differences
of bounded rotation/direction products. Unrepresentable motion/construction raises
`OpenCV_Error`; there is no numerical-unavailability result with a fabricated E.

If motion reports exact baseline zero, `Known_Pose_Essential_Result.Defined=False`;
its relative pose and zero physical baseline remain available but its variant has
**no** matrix, rotation-matrix, or direction fields. This includes pure rotation
about an exactly common center. No zero translation is normalized. No baseline
epsilon exists: representable tiny nonzero baselines remain nonzero. A positive
baseline with an exactly zero representable relative translation raises
`OpenCV_Error`. Roundoff in supplied poses/native transforms is not silently
reinterpreted as an exact common center.

## Input validation and calibration

`Verify_Known_Pose_Epipolar` accepts ordinary pixel arrays, separate intrinsics,
separate five-coefficient Brown models, two poses, and mandatory policy. Focal
lengths must be finite positive; principal points, all Brown coefficients, all
pose components, and pixels must be finite. Counts must match and fit native
INT32. All inputs and policy fields are validated even for empty arrays.
`Minimum_Correspondences` is an Ada `Positive`; supplying zero is outside that
type's contract. No numerical policy field has a default. Both limits must be
finite nonnegative normalized-image-plane quantities, **not pixels**.

After validation, a valid count below the caller minimum returns immediately
without *any* native evaluation. Geometry and summaries are unavailable. Otherwise
geometry is evaluated; exact zero baseline returns before undistortion or scoring.
For defined E, call native-backed `Undistort_To_Normalized` separately for each
camera, then public `Normalized_Sampson_Error` once for **every** positional pair.
Input lower bounds need not agree, including singleton and multiple-element
arrays ending at `Positive'Last`. Owned output arrays are always `1 .. Count`, in
original pair order. Empty arrays are `1 .. 0`.

Undistortion retains the existing fixed COUNT|EPS, 20 iterations, 1e-12
pixel-residual termination contract. It is approximate; an extreme/noninvertible
Brown model has no new convergence or uniqueness guarantee. Distorted pixels
never enter the normalized metric directly.

## Per-pair and whole-set diagnostics

The existing public native-backed metric is used unchanged:

```text
a = E x1; b = E^T x2; r = x2^T E x1
error = abs(r) / sqrt(a.x^2 + a.y^2 + b.x^2 + b.y^2)
```

`Epipolar_Correspondence_Result.Defined=False` has neither an error nor a point
threshold flag. Zero denominator (e.g. an exact epipole) or unrepresentable native
metric arithmetic is undefined, **not zero**. All other pairs are still evaluated.
Each defined value must be finite nonnegative and has `Passes_Point_Limit` equal
to the direct inclusive comparison with `Maximum_Point_Normalized_Sampson`.

Whole-set RMS and maximum are published only if every supplied pair is defined.
Scaled sum-of-squares avoids squaring unscaled large errors; divide by count
before multiplying by scale. The bounded SSQ/count ratio is capped at one solely
to contain rounding at the representable RMS range boundary, as in the existing
reprojection summarizer. No threshold comparison is scaled, subtracted, or
modified by an empirical epsilon.

The result is a private owned discriminated-by-count Ada value. Its accessors:

* `Disposition`, `Accepted`, and original `Count` are always available.
* `Geometry_Evaluated` guards `Geometry`, which raises `OpenCV_Error` if unavailable.
  The returned geometry itself uses `Defined` to guard Essential coefficients.
* `Correspondence_Errors` always returns one entry per original pair. On skipped
  evaluation these entries are undefined, not measured zero scores.
* `Entire_Set_Evaluable` guards `Summary`; requesting an unavailable or partial
  summary raises `OpenCV_Error`. Summary fields explicitly say normalized Sampson.

| Disposition | Available evaluation |
|---|---|
| `Insufficient_Correspondences` | Input validation only, no native entry. |
| `No_Physical_Baseline` | Relative pose/baseline, no E, errors or summary. |
| `Undefined_Correspondence_Errors` | E and every available pair diagnostic, no whole-set summary. |
| `Rejected_By_Policy` | E, all errors and whole-set summary, one or more limits fail. |
| `Accepted` | Every error defined, count sufficient, RMS and maximum within limits, every point passes. |

Native exceptions propagate as `OpenCV_Error`. No partially constructed result
is published on failure. Temporary Mats remain Core-owned and handles are
borrowed only inside `Module_Interop` callbacks. No native pointer, result handle,
or borrowed memory escapes; copying a result copies owned values.

## Task 021 adapter

The overload accepting two `Camera_Relocalization_Result` values requires both
`Accepted=True`, even for empty cross-image arrays. It retrieves both poses only
through `Accepted_Pose` and delegates to the primary operation. It never reruns
RANSAC/refinement and contains no duplicate Essential or scoring algorithm.

Cross-image correspondences are separately supplied and may refer to completely
different features than either fitting/held-out set. The caller is responsible
for provenance: the API does **not** certify independence from pose fitting,
calibration, reconstruction, or matching errors.

## Observability limits

Magnitude-only epipolar residuals do not verify absolute position or physical
baseline magnitude, cannot distinguish E from -E, do not establish positive camera
depth or 3D reconstruction accuracy, and do not prove matches are correct.
Calibration/matching errors may be shared with pose estimation. Acceptance means
only consistency under the supplied calibration, poses, matches, and caller
policy. This is not localization uncertainty estimation, navigation fusion,
physical ground truth, or evidence from sensors. No covariance, bundle adjustment,
visual odometry, SLAM, DTED, geographic coordinates or navigation API is added.