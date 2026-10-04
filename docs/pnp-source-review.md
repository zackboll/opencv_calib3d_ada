# PnP source review

Official annotated tags were fetched with `git ls-remote` and their peeled
revisions verified during Task 001. `source-provenance.json` records immutable
raw URLs, SHA-256 and Git blob SHA-1 for each reviewed header, solvepnp, epnp and
projection/Rodrigues implementation. No mutable branch URL is provenance.

| OpenCV | Peeled revision | Native area |
|---|---|---|
| 4.1.0 | `371bba8f54560b374fbcd47e7e02f015ac4969ad` | `modules/calib3d` |
| 4.10.0 | `71d3237a093b60a27601c20e9ee6c3e52154e8b1` | `modules/calib3d` |
| 5.0.0 | `40738fb16ceddb5fb3fea747585f7ce6abb0605b` | `modules/geometry` |

## Reachable fixed-policy behavior

The common pinhole declaration is in `calib3d.hpp` at lines 811 (4.1),
1014 (4.10), and `geometry/3d.hpp` at line 1090 (5.0). The later fisheye
overload in the 5.0 header is **not** the function this shim calls.

The legacy `solvePnPRansac` body starts at solvepnp.cpp lines 260/202/172 for
4.1/4.10/5.0 respectively. It converts Float64 correspondences to Float32 before
RANSAC, checks contiguous point vectors with `checkVector(3)`/`checkVector(2)`,
requires matching counts >=4, and creates 3x1 Float64 scalar pose outputs.
Our callback-owned Nx1 Float64 C3/C2 point matrices satisfy those schemas.
Consequently Float64 binding inputs do **not** imply Float64 RANSAC arithmetic.

For our fixed `SOLVEPNP_EPNP`, `useExtrinsicGuess=false` policy:

* The usual minimal sample has **five** points and uses EPNP.
* With **exactly four** input correspondences, upstream changes the kernel to
  **P3P**, calls `solvePnP` directly, bypasses the consensus loop, and on success
  marks all four correspondences as inliers. This is not four-point robust
  outlier rejection. Exactly five similarly uses direct EPNP and all five.
* With more than five, the RANSAC callback obtains candidate poses via
  `solvePnP`, and computes pixel residuals through `projectPoints`. A consensus
  mask is compressed into correspondence vectors. The final solve uses the
  requested EPNP flag; iterative/DLT fallback code in newer revisions is not
  reachable with this binding's policy.
* EPNP's solve path normalizes image coordinates with `undistortPoints`, uses
  the double camera matrix in `epnp`, computes R/t, and applies `Rodrigues` to R.
  This implementation does not justify adding a noncoplanarity Ada heuristic.
* Failed direct solve, failed consensus, or failed final solve returns false and
  releases native inliers. Native pose vectors may then be arbitrary; the shim
  deliberately ignores them and publishes an owned no-pose result instead.
* Success constructs CV_32S zero-based inliers from ascending mask positions.
  The shim independently validates count >=4 and <=N, range, and uniqueness,
  sorts, then Ada converts each index to one-based ownership.

Newer sources also contain USAC dispatch and other solve methods. They are not
selected by the fixed flag and are not part of this source qualification.

## Projection and camera-center implementation

OpenCV 4.1/4.10 use `calibration.cpp`: public `projectPoints` begins at
3552/3714, `Rodrigues` at 3479/3635, and the legacy projection kernel at
520/522. OpenCV 5.0 moved these to **geometry/src/calibration_base.cpp**:
public projection at 1432, kernel at 505, Rodrigues at 121.

Projection checks Float32/Float64 object vectors, creates Nx1 C2 output with
the input depth, and computes `X_camera = R * X_world + t`, perspective division,
Brown radial/tangential distortion, then focal scaling and principal-point
addition. The supplied 3x3 CV_64F camera matrix has zero skew; five CV_64F
distortion coefficients are accepted and ordered `(k1,k2,p1,p2,k3)`.
The 5.0 kernel can use a HAL path but preserves the same schema/model.
The shim uses Float64 exclusively and validates finite output before publication.

Rodrigues accepts the 3x1 Float64 rotation vector, preserves depth, returns the
3x3 matrix, and implements the axis-angle formula (identity for vanishing angle).
The shim derives `C_world = -R^T * t`; it does not interpret t as camera position.

Independent test oracles are pinhole identity `(20,60)`, translated `(30,20)`,
Rz(pi/2) projection `(-10,40)`, all-five distortion
`(10.03350625,20.1220125)`, and Rz(pi/2) camera center `(-2,1,-3)`.
These expected values are mathematical fixtures, not calls back into the binding.

Pose refinement and other estimator policies remain outside this baseline.
