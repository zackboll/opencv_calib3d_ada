# PnP contract

## Frames

The binding follows OpenCV's extrinsic convention exactly:

```text
X_camera = R * X_world + t
```

`Rotation` is an axis-angle Rodrigues rotation vector. `Translation` is `t`.
It is not a camera position. The camera center expressed in the world frame is:

```text
C_world = -R^T * t
```

`Camera_Center` uses native OpenCV `Rodrigues` to construct `R` and applies that
formula. Tests include a non-identity rotation so an incorrect `-t` shortcut
cannot pass.

World is the caller's metric Cartesian frame, not a geographic definition.
ENU/NED, geodetic and terrain conversions belong outside this crate.

## Intrinsics and distortion

The initial intrinsic profile is:

```text
[ fx  0  cx ]
[  0 fy  cy ]
[  0  0   1 ]
```

`fx` and `fy` are finite and positive; principal point coordinates are finite.
Skew is deliberately fixed to zero. Distortion is the 5-term OpenCV/Brown vector
`(k1,k2,p1,p2,k3)`. Rational, thin-prism and tilted models are deferred.
All five coefficients must be finite.

## Robust pose

`Solve_PnP_RANSAC` passes `SOLVEPNP_EPNP`, `useExtrinsicGuess=false`, an explicit
iteration limit, pixel reprojection threshold and confidence. The public profile
requires at least four correspondences because that is the common OpenCV public
minimum. Upstream OpenCV has special internal behavior for exactly four points;
the source review confirms that exactly four uses direct P3P, and exactly five
uses direct EPNP; each successful direct path marks all input points as inliers.
More than five uses five-point EPNP RANSAC samples and final EPNP consensus solve.
The internal P3P special case is not a public solver selector.

Iterations must be positive and <=INT32_MAX; threshold must be finite, positive
and remain finite and positive after native float conversion; confidence must be
finite and strictly between zero and one. Correspondence arrays must match,
contain at least four points, and have only finite coordinates. There is no
Ada-layer geometric noncoplanarity heuristic.

A native `false` return is an expected estimation outcome, not an ABI error.
`Found=False` exposes no pose and no inliers. `Pose` raises `OpenCV_Error` when
called on such a result.

Successful inliers are unique, valid, sorted and converted from OpenCV zero-based
indices to Ada one-based correspondence indices.
Malformed successful native results become an error, not a partial estimate.
No-pose `Inliers` has bounds `1 .. 0` and count zero.

At the raw ABI, scalar/result outputs are cleared before validation. Projection
uses a temporary native matrix and publishes only after successful validation;
failure leaves the borrowed caller output unchanged. No native handle is retained.
