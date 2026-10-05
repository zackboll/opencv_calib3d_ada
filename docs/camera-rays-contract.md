# Camera rays and frame transforms

```text
distorted pixel -> normalized camera coordinate -> camera ray -> world ray
```

All outputs are ordinary Ada values. Core owns the temporary Mats; callback-scoped
handles never escape. The caller defines a metric Cartesian world frame, not ENU,
NED, ECEF, geodetic coordinates, or a terrain/Earth model.

| Quantity | Type | Units |
|---|---|---|
| distorted input | `Image_Point` | pixels |
| undistorted normalized coordinate | `Normalized_Image_Point` | dimensionless |
| camera bearing | `Camera_Direction` | dimensionless unit vector |
| world bearing | `World_Direction` | dimensionless unit vector |
| ray origin | `Object_Point` | caller's world-coordinate units |

`Normalized_Image_Point` and direction types derive from Core's Float64 numeric
vectors but retain semantic distinctions. A normalized point `(x,y)` represents
the projective camera direction `(x,y,1)`, not a pixel or a 3-D point.

## Undistortion

`Undistort_To_Normalized` calls standard `cv::undistortPoints`, never fisheye.
Intrinsics have zero skew and positive finite fx/fy. Distortion is exactly the
five Brown coefficients K1 K2 P1 P2 K3. R and P are omitted: no rectification or
new camera matrix. Zero distortion gives `(u-cx)/fx, (v-cy)/fy`. The native call
uses **COUNT | EPS, 20 iterations, 1e-12 pixel-residual epsilon**, not upstream
version-dependent defaults. No public iteration tuning knob is exposed.

For zero coefficients the shim passes an empty distortion Mat, OpenCV's documented
zero model, avoiding meaningless `0*r6` overflow for large finite ideal points.
Nonzero coefficients use a 5x1 CV_64F Mat. Inversion is approximate, may fail to
converge for extreme/noninvertible distortion, and does not promise a universal
error bound or unique inverse. A finite native approximation is returned, not a
new convergence-status API. The qualified moderate all-five forward Brown fixture
uses **1e-10 dimensionless component tolerance**, comfortably above 1e-12 pixel
termination/focal scaling and Float64 rounding; results need not be bit-identical.

All image APIs preserve input length/order and return one-based arrays. Empty
input returns `1 .. 0` **after validating camera/distortion and, for world rays,
all six pose components**, consistent with `Project_Points`. Invalid parameters
are rejected even for empty input. Valid empty calls do not enter native
undistortion; empty world rays do not compute Rodrigues or the camera center.

Private undistortion accepts Nx1 CV_64FC2, snapshots valid noncontinuous Core
Regions, stages output in a local Mat, verifies schema/count/all coordinates
finite, then rebinds the destination. Failure leaves its header/data unchanged.
Input/output handles can alias safely because publication follows computation.

## Pose and transforms

```text
R = Rodrigues(Pose.Rotation)
X_camera = R * X_world + t
X_world = R^T * (X_camera - t)
C_world = -R^T * t
d_camera = R * d_world
d_world = R^T * d_camera
```

`Rotation_Matrix_Of` uses native Rodrigues (no Ada reimplementation, no Jacobian).
The public 0..2 by 0..2 array is independent of the private nine-double C record.
Whole-pose finiteness is required even though translation does not enter
Rodrigues/direction arithmetic. Point inputs/intermediates/results must be finite.
Direction transforms require finite components and **preserve magnitude without
normalizing**, including zero directions. Translation has no effect on them.
Unrepresentable arithmetic raises `OpenCV_Error`.

## Bearings

`Camera_Bearing_Rays` removes intrinsics/distortion first, then normalizes `(x,y,1)`.
Scaled normalization divides by the largest absolute component, computes the
bounded scaled norm, and divides again without forming an overflowing unscaled
norm. Returned camera directions are finite, unit length within **1e-12 Float64
norm tolerance**, and have positive Z. A nonpositive/underflowed Z is rejected.

`World_Bearing_Rays` computes R once, rotates each camera unit direction by **R^T**,
and robustly normalizes again to absorb rotation rounding. Every origin is exactly
the same copied `Camera_Center(Pose)` value; each direction is finite and unit.

```text
X_world(s) = C_world + s * d_world
s > 0
```

Positive s follows the forward camera half-ray. s has the caller's world-coordinate
units and is meters only if that frame uses meters. Translation, camera center,
ray origin, direction, and point are distinct quantities. A ray does not establish
range or terrain intersection. No terrain, geographic conversion, estimator,
triangulation, Features dependency, or image remap is added.

## Independent qualification oracles

* K=(100,200,10,20), pixel=(110,20): normalized=(1,0), camera=(1,0,1)/sqrt(2).
* Principal pixel=(cx,cy): normalized=(0,0), camera=(0,0,1).
* Rodrigues zero -> identity; rvec=(0,0,pi/2) -> `[[0,-1,0],[1,0,0],[0,0,1]]`.
  Rows/columns, R^T R and det=+1 are tests, not new native rejection criteria.
* Rz(pi/2), t=(1,2,3): world point=(1,0,0) -> camera=(1,3,3), inverse recovers
  (1,0,0); camera center maps to zero.
* World X -> camera Y; camera X -> world -Y; changing t leaves directions unchanged.
* Off-axis world ray: origin=(-2,1,-3), direction=(0,-1,1)/sqrt(2).
* All-five independently forward-distorted (0.3,-0.2) -> world direction
  `(-0.2,-0.3,1)/sqrt(1.13)`, not an expected value obtained from the binding.

See `camera-rays-source-review.md` for immutable native evidence and
`bootstrap-validation.md` for actual execution, not merely test inventory.