# Known-pose plane-induced homography (Task 027)

Nothing is fitted or estimated. The caller supplies two world-to-camera poses,
one infinite Cartesian world plane and, for ideal-pixel conversion, two independent
zero-skew intrinsic calibrations. All geometry uses one caller-defined world frame
and length unit. Outputs are owned Ada values; no handles or native storage escape.
The private ABI remains 48 operations, with no C++ or dependency addition.

```text
Two supplied world-to-camera poses
              |
      Relative R21 and t21
              |
        Supplied world plane
              |
  Plane normal and signed offset
              |
  Normalized plane-induced H12
              |
       +------------------+
       |                  |
Normalized point      K2 * H * K1^-1
transfer              ideal-pixel matrix
       |                  |
       +------------------+
              |
      Planar image geometry
```

## Frames, orientation and projective scale

```text
X1 = R1 * X_world + t1       X2 = R2 * X_world + t2
C1 = -R1^T * t1             C2 = -R2^T * t2
R21 = R2 * R1^T             t21 = t2 - R21 * t1
X2 = R21 * X1 + t21
n_world . (X_world - Anchor) = 0
n1 = R1 * unit(n_world)     d1 = unit(n_world) . (Anchor - C1)
n1^T * X1 = d1
p2 ~ H12 * p1               p = (x_normalized, y_normalized, 1)
H12 ~ R21 + t21 * n1^T / d1 ~ d1 * R21 + t21 * n1^T
```

**Translation t21 is expressed in Camera 2, not a world-frame camera-center
displacement.** Neither t1 nor t2 is camera position. Relative motion, camera centers,
Rodrigues matrices and direction conversion reuse existing geometry APIs. The plane
normal uses the existing scaled normalization helper, without squaring its original
magnitude. Negating the normal reverses both normal and signed offset; positive
finite rescaling and normal sign reversal preserve projective mapping to Float64
rounding when their required terms remain representable.

`Construct_Known_Plane_Homography` returns `Known_Plane_Homography_Result`.
`Disposition` and `Defined` are always readable. `Normalized_Homography` raises
`OpenCV_Error` unless disposition is `General_Plane_Homography` or
`Pure_Rotation_Homography`. There are no public absence-indicating coefficients.
The existing `Homography_Matrix` has arbitrary nonzero common scale: **h22 need not
be one, or even nonzero**. Compare mapped coordinates, not coefficient scale.

## Exact geometric dispositions

* Exact common physical centers, as identified by
  `Measure_Camera_Motion.Baseline_Direction_Defined=False`, give
  `Pure_Rotation_Homography`, H12 ~ R21. This precedes plane-offset evaluation.
  A plane containing that common center is allowed. This algebraic map does
  **not** establish positive physical depth or a valid forward ray-plane intersection.
* Otherwise exact computed d1=0 gives `Plane_Through_First_Camera_Center`.
* Otherwise exact computed unit(n_world).(Anchor-C2)=0 gives
  `Plane_Through_Second_Camera_Center`.
* Otherwise checked construction gives `General_Plane_Homography` or
  `Homography_Numerically_Unavailable`.

Neither through-center case is published as an ordinary model; if both centers
lie on the plane the first-center disposition takes precedence. There is no epsilon
on offset, baseline, denominator or coefficient magnitude. A tiny nonzero baseline
is not pure rotation. Exact means the represented calculation, not an assertion
about unobservable real-world geometry.

## Finite Float64 construction and error boundaries

All plane anchor/normal and pose components must be finite; the normal must be
nonzero. Validate these before native entry. Invalid inputs raise `OpenCV_Error`.
Existing native inversion/composition, Rodrigues, camera-center and direction APIs
remain fallible: their errors propagate as `OpenCV_Error`, never as routine absence.
Unrepresentable prerequisites rejected by those existing APIs also propagate; this
API does not guess whether a native error denotes injected failure or range failure.

After those prerequisites, compute both signed offsets using checked subtraction,
product and sum. The matrix uses:

```text
s = max(abs(d1), abs(t21.x), abs(t21.y), abs(t21.z))
a = d1/s       b = t21/s
H = a*R21 + b*n1^T
```

Reject zero scale, nonfinite differences/products/sums/divisions, nonzero products
or quotients underflowed to zero, and an all-zero matrix. Finally divide by maximum
absolute coefficient, again checking loss of nonzero coefficients. Never impose an
h22 normalization, saturate a coefficient or publish NaN/infinity. Arithmetic-only
helpers raise a dedicated local exception, caught only around Ada arithmetic.
No native call or guarded accessor is inside those handlers. Failed function
assignment cannot replace an earlier assigned result.

This is conservative finite Float64 arithmetic, not an extended-precision projective
solver: intermediate overflow/underflow can reject mathematically representable
symbolic results. Cancellation and rounding remain possible. Large world-coordinate
origins can erase small geometric differences before any check can detect them;
use appropriate local Cartesian coordinates. Near-center planes and near-horizon
transfer are ill-conditioned. Normalization retains the existing Float64 direction
contract, not arbitrary-precision direction recovery.

## Three distinct coordinate domains

1. **Brown-distorted observed pixels:** camera-specific five-term distortion.
   First use `Undistort_To_Normalized` independently for each camera as appropriate.
2. **Ideal pinhole pixels:** undistorted coordinates with focal/principal-point
   scaling applied. `Ideal_Pixel_Homography` computes **K2 * H12 * inverse(K1)**.
3. **Dimensionless normalized image coordinates:** (x,y) describes (x,y,1).
   `Normalized_Homography` and `Transfer_Normalized_Plane_Points` use this domain.

`Ideal_Pixel_Homography` validates both calibrations with the existing finite,
positive-focal validator. Different focal lengths and principal points are supported.
Scaled K matrices and a scaled adjugate of K1 avoid unnecessary focal reciprocals;
checked matrix products and scaling reject unrepresentable coefficients with
`OpenCV_Error`. No distortion coefficients enter this formula. **There is no single
exact homography for arbitrary raw Brown-distorted image pixels.** The ideal-pixel
matrix must not be applied to them as if it were one.

## Normalized batch transfer

`Transfer_Normalized_Plane_Points` first checks model availability, then validates
**every** normalized point before the first mapping. It delegates each evaluation
to the existing pure-Ada `Map_With_Homography`. No native operation is introduced
by algebraic transfer. `Normalized_Plane_Transfer.Finite=True` alone permits reading
its `Point : Normalized_Image_Point` variant. Zero homogeneous denominator or
unrepresentable mapped coordinates yield `Finite=False`, not a successful (0,0).

Results preserve positional order and have bounds 1..N, including empty 1..0.
Rebased inputs and multiple elements ending at `Positive'Last` use
`Points'First + (I - 1)`, avoiding intermediate index overflow. Empty input still
requires an available model. Invalid point inputs raise `OpenCV_Error`, not per-point
unavailability. Mapping does not independently certify that the model is invertible.

This is **algebraic projective mapping**. It does not test positive depth, image
boundaries, finite plane-patch membership, forward-ray range, or physical acceptance.
Task 024 [ray-plane](world-plane-intersection-contract.md), Task 025
[forward](plane-correspondence-contract.md), and Task 026
[reciprocal](reciprocal-plane-correspondence-contract.md) APIs remain authoritative
for their separate acceptance semantics. Correct projective transfer alone cannot
prove reciprocal acceptance.

One homography describes **one plane**, not arbitrary nonplanar terrain. No image
loading, warp, resampling, DTED/DEM lookup, geodesy, feature matching, pose/plane
fitting, RANSAC, nonlinear refinement, navigation fusion or uncertainty estimator is
added. Synthetic observations establish no physical terrain-registration or
navigation accuracy. See [qualification](known-plane-homography-validation.md).