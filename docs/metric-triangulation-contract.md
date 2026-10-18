# Calibrated metric two-view triangulation (Task 019)

`Triangulate_World_Points` reconstructs owned Ada values from paired distorted
pixels, independent intrinsics and five Brown coefficients per camera, two known
world-to-camera poses, and mandatory `Metric_Triangulation_Options`.
No new solver, native operation, dependency, or C++ change: private ABI remains
48 operations. Essential-matrix recovery is not invoked.

## Frames and scale

Both poses MUST use the same caller-defined Cartesian world frame and length
units. Translation is not camera position:

```text
X_first = R1 X_world + t1       C1_world = -R1^T t1
X_second = R2 X_world + t2      C2_world = -R2^T t2
R_relative = R2 R1^T
t_relative = t2 - R_relative t1
B = norm(C2_world - C1_world)
```

`Measure_Camera_Motion` obtains existing `Camera_To_Camera_Pose`, itself
`Compose_Poses (Invert_Pose (First), Second)`, and a scaled physical baseline norm.
Normalize **t_relative**, preserving sign, with the existing scaled unit-vector
operation. This term is in the second-camera frame. World-frame camera displacement
has a different frame and sign; substituting it is wrong. Essential-matrix
translation alone supplies no baseline magnitude. Here scale comes entirely from
the two supplied known absolute poses.

```text
Distorted pixels
 -> Undistort_To_Normalized separately with each camera's calibration
 -> Triangulate_Normalized: P1=[I|0], P2=[R_relative|normalize(t_relative)]
 -> X_first_metric = B * X_first_unit; Z1_metric=B*Z1_unit; Z2_metric=B*Z2_unit
 -> Camera_To_World_Point: X_world = R1^T (X_first_metric - t1)
 -> Project_Points into BOTH original calibrated, distorted cameras
```

An arithmetic-only private overload of `Camera_To_World_Point` shares the public
transform implementation and accepts a previously obtained native rotation matrix.
Native Rodrigues is never inside a per-point arithmetic-failure handler.

## Validation and results

Validate finite pixels, intrinsics (positive focal lengths), distortion, all six
pose components, counts equal and fitting INT32, and all options before
reconstruction. Pair positionally, not by index equality. Outputs are `1..N`,
including `1..0` for valid empty requests. Add indices as `First+(I-1)` so
`Positive'Last` bounds do not overflow. Empty requests validate inputs but do not
calculate motion or enter native operations.

Exact zero baseline produces `No_Physical_Baseline` per pair without native
triangulation, source status, coordinates, depths, or residuals. No baseline
epsilon is used. Nonzero baseline is not silently collapsed. Task 018 may itself
raise on unrepresentable motion arithmetic.

`Metric_Triangulated_Point` separates `Disposition` from three discriminated
optional records. Read fields only if the record's `Available` is true:

* `Source`: original `Triangulated_Point`, including status, and existing
  `Assess_Triangulation` quality/parallax. Present whenever triangulation ran.
  Source XYZ/depth/error fields exist only for source `Status=Usable`.
* `Geometry`: metric first-camera and world positions and both scaled depths.
* `Pixels`: finite Euclidean reprojection errors in pixels and comparison flags.

Source infinity, unrepresentability, nonpositive depth, and undefined normalized
reprojection map to `Source_At_Infinity`, `Source_Unrepresentable_Point`,
`Source_Non_Positive_Depth`, and `Source_Undefined_Reprojection`. They never expose
absent source fields or publish geometry. `Metric_Conversion_Unavailable` retains
source diagnostics when per-point Ada scaling, transformation, actual world-point
depth verification, or pixel residual arithmetic cannot be represented. Geometry
may remain available when only pixel residuals are unavailable. No zero placeholder
represents a reconstructed point.

Every scaled coordinate/depth must be finite; nonzero components scaling to exact
zero and nonpositive depths are rejected. Actual camera depths of the representable
world point are rechecked before projection. Overflowing intermediates are rejected
even if later cancellation could mathematically yield a finite answer. Residual
norms use scaled arithmetic. No saturation or clamping occurs.

Native undistortion, triangulation, Rodrigues, motion, and projection failures
propagate as `OpenCV_Error`, not routine rejection. The complete function returns
or an exception propagates, without partial publication. Core owns temporary Mats;
native borrowing remains callback scoped.

## Caller policy and units

All four thresholds are required, with no default empirical cutoff:

* `Minimum_Acute_Parallax_Radians`: finite, 0..pi/2.
* `Maximum_Normalized_Reprojection_Error`: finite, nonnegative, dimensionless.
* `Maximum_First_Pixel_Reprojection_Error`: finite, nonnegative, pixels.
* `Maximum_Second_Pixel_Reprojection_Error`: finite, nonnegative, pixels.

`Assess_Triangulation` uses `Measure_Stereo_Parallax`'s acute line-intersection
angle, NOT forward directed ray angle. Acceptance requires angle >= minimum,
both normalized errors <= shared maximum, and each pixel error <= its own maximum.
Equality passes without epsilon. Source `Usable` can be `Metric_Policy_Rejected`
with full geometry/diagnostics retained. Only `Metric_Accepted` passes the whole
policy. This is not covariance, confidence, a correspondence certificate, or a
3-D accuracy bound.

## Representation limitations

Iterative undistortion is approximate, without a universal inverse-distortion
accuracy bound. Near-parallel rays can yield huge ill-conditioned finite points.
Tiny physical baseline is not zero geometry. Float64 rounding at large world
origins can erase small offsets despite finite arithmetic. No arbitrary world
accuracy tolerance is imposed: callers must choose a suitable local Cartesian
frame and policy. Coordinates are meters only if that frame uses meters. No
geographic conversion, navigation fusion, or uncertainty model is supplied.
