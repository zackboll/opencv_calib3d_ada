# Essential estimation and relative pose

**Essential geometry requires calibrated normalized coordinates. Do NOT pass raw
distorted pixels to `Estimate_Essential_RANSAC`.** Inputs are the existing
`Normalized_Image_Point_Array`, not pixel `Image_Point_Array`. Convert each
camera separately with Task 003:

```ada
First_Normalized := Undistort_To_Normalized
  (First_Pixels, First_Intrinsics, First_Distortion);
Second_Normalized := Undistort_To_Normalized
  (Second_Pixels, Second_Intrinsics, Second_Distortion);
```

This supports a camera over time or different camera intrinsics/distortion,
without an Essential API camera-model dependency. There are no pixel/intrinsics
convenience overloads. Native estimation and recovery both use identity K.

## Essential matrix and robust profile

`Essential_Matrix` is a distinct 3x3 Ada Float64 value:

```text
x2^T * E * x1 = 0
x1 = (normalized_x1, normalized_y1, 1)
x2 = (normalized_x2, normalized_y2, 1)
```

E has arbitrary nonzero common scale. Neither `E(2,2)=1` nor any coefficient
sign convention is promised. Do not compare estimated coefficients directly
across native versions. The five-point solver establishes Essential structure;
the binding adds no fragile rank/singular-value rejection threshold.

`Estimate_Essential_RANSAC` requires equal finite counts **>=6**, fitting INT32.
OpenCV supports a five-point minimal solve: exactly five directly calls the
kernel with all points and an all-ones mask, bypassing subset consensus. The
six-point minimum is deliberately a binding policy so an API named RANSAC
actually uses random subset consensus. Ada and raw C independently reject 0..5
before native entry; armed stage 29 verifies rejection/entry ordering.

The binding fixes the native maximum at **1000**;
adaptive RANSAC termination may stop earlier. There is no iteration option,
LMEDS, USAC, or arbitrary estimator selection.
OpenCV 4.x uses the legacy camera-matrix overload (4.1 registrator default 1000;
4.10 compatibility overload forwards with 1000). OpenCV 5.0 removed that
overload, so its required maxIters argument is explicitly the fixed literal
1000. This compile-time syntax difference does not vary the public policy.

`Essential_RANSAC_Options` defaults:

* `Normalized_Epipolar_Threshold = 1.0E-3`, dimensionless, NOT pixels;
* `Confidence = 0.999`.

The shared `essential_profile.hpp` qualifies IEEE binary64/binary32. Threshold
must be finite positive, its double square finite, and the square rounded to
binary32 finite positive (neither underflow-to-zero nor overflow-to-infinity).
Confidence must be finite and strictly between 0 and 1. Unlike fundamental
estimation there is no DBL_EPSILON boundary substitution with 0.99. Ada preflight
uses the same private C validator, not independently duplicated rounding math.

**Essential observations and five-point algebra are Float64.** The callback
calculates its Sampson-like squared error in Float64 but stores `CV_32F`; generic
RANSAC compares that to `float(double(threshold*threshold))`. This native mask
boundary still affects model selection, but is NOT the public final-inlier policy.

## Public normalized Sampson metric and final-E inliers

`Normalized_Sampson_Error` returns the square root of the reviewed callback
quantity, NOT Task 005 `Maximum_Epipolar_Error` (maximum of two point-to-line
errors):

```text
a = E * x1; b = E^T * x2; r = x2^T * E * x1
Error = abs(r) / sqrt(a.x^2 + a.y^2 + b.x^2 + b.y^2)
```

The error is dimensionless normalized-camera-plane error. Checked products/sums
and nested scaled `hypot` avoid naïvely squaring arbitrary finite values.
All E coefficients and both points must be finite; otherwise `OpenCV_Error`.
Zero denominator or nonrepresentable required multiplication/sum, residual,
norm or final division returns `Defined=False`. No denominator epsilon,
clamping, NaN return, or escaping `Constraint_Error`.

After a finite nonzero exactly 3x3 `CV_64FC1` final native E, classify every
ORIGINAL normalized Float64 correspondence: public Essential inlier iff defined
Sampson error <= the requested threshold. Publish in correspondence order.
Malformed nonempty E is a native/ABI error, not a no-model result.

`Found` means a supported E: at least **5 final Essential inliers**. Empty native
E or fewer than five final inliers gives success/no-model: `Found=False`, E
unavailable, empty Essential and pose inliers, `Pose_Recovered=False`.
`Essential` raises `OpenCV_Error` on no-model. Inlier access is one-based;
positions are valid, unique and strictly ascending. Empty arrays are `1 .. 0`.

## Automatic pose recovery, frames and unknown scale

For supported E, construct a `CV_8U` mask containing **exactly final-E public
inliers**, not the `findEssentialMat` native mask. `recoverPose` intersects its
four triangulation/positive-depth hypotheses with this mask and selects maximum
cheirality support. The binding passes explicit
`std::numeric_limits<double>::max()` as the effectively-unbounded finite depth
threshold, not the default overload's hidden 50 baseline units. Infinite or
nonrepresentable internal triangulations can still fail cheirality. No
triangulated points are exposed.

`Relative_Camera_Pose` is semantically distinct from `World_To_Camera_Pose`:

```text
X_second = R * X_first + lambda * t_hat
R = Rotation_First_To_Second
t_hat = Translation_Direction, norm(t_hat)=1
lambda > 0 is UNKNOWN baseline magnitude
```

**recoverPose does NOT recover metric baseline magnitude.** It recovers relative
rotation and translation direction only, not visual-odometry scale or metric
navigation position. All native R/t components are validated finite; t must be
nonzero and is robustly normalized before publication. SO(3) orthogonality,
positive determinant, and unit direction are qualification checks, not fragile
production tolerances.

**Translation_Direction is not camera position or camera-center direction.** It
is the translation TERM in the first->second transform. For camera motion in
first-camera coordinates use:

```text
Second_Camera_Center_Direction_In_First(Pose)
    = normalize(-R^T * t_hat)
```

The helper validates finite inputs/nonzero resulting direction. A user-created
record is not automatically certified as a rotation matrix.

`Pose_Recovered=True` requires **>=5 cheirality-qualified correspondences**.
`Recovered_Pose` otherwise raises `OpenCV_Error`. Pose inliers are valid, unique,
ascending and a subset of Essential inliers. Insufficient cheirality support
does NOT discard E: `Found=True`, E/support accessible, no public pose/inliers.

## Ownership and limits

Public results are limited controlled Ada values owning only copied value/index
data. Native result handles remain private and are destroyed under a guard.
Raw normalized schema: equal Nx1 `CV_64FC2`, finite, >=6, INT32 count. Both
noncontinuous Core Regions are accepted and snapshotted before native reshaping;
no borrowed Core handles survive callbacks. All raw outputs clear before
fallible work; destroy(null) is safe; exceptions cannot cross C ABI.

No public triangulation, decomposition, calibration, stereo rectification,
fisheye, Features dependency, navigation fusion, geospatial logic or generators
are included. Low parallax, degeneracy and incorrect calibration can still
produce misleading geometry: five support is a conservative publication policy,
not an observability or uncertainty guarantee.