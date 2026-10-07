# Robust image-space homography contract

**A single homography models a projective mapping between planes.** It is
appropriate for planar scenes, approximately planar local regions, and projective
image-registration/correspondence-verification models. It is **not a general 3-D
terrain pose model**. Do not apply a global homography to strongly nonplanar
terrain as a localization substitute. For actual 3-D world points, use the
existing PnP pose API.

## Mapping and scale

`OpenCV.Calib3D.Homography_Matrix` is a semantic 3x3 Float64 Ada array:

```text
[x']     [h00 h01 h02] [x]
[y']  ~  [h10 h11 h12] [y]
[1 ]     [h20 h21 h22] [1]

w  = h20*x + h21*y + h22
x' = (h00*x + h01*y + h02) / w
y' = (h10*x + h11*y + h12) / w
```

A homography is defined only up to a nonzero common scale. **There is no public
`h22 = 1` invariant.** Compare projective mappings, not coefficient equality.

`Map_With_Homography` is pure Ada. Matrix entries and point components must be
finite; invalid inputs raise `OpenCV_Error`. The discriminated
`Homography_Point_Result` contains `Point` only when `Finite=True`. Zero `w`,
nonfinite arithmetic, or overflow yields `Finite=False`. A merely small nonzero
denominator is not rejected, and coordinates are never clamped. A zero matrix
maps no finite point; mapping does not perform native estimation validation.

## Robust-only estimation

`Estimate_Homography_RANSAC(Source_Points, Destination_Points, Options)` requires
equal counts **>=5**, representable by INT32, and finite coordinates in both
arrays. **OpenCV's own n=4 findHomography path bypasses RANSAC. The binding's
robust API therefore intentionally requires >=5.** Both Ada and the raw ABI
reject 0..4 before native entry. There is no public four-point direct solver.

`Homography_RANSAC_Options` defaults:

| Field | Default | Validation |
|---|---:|---|
| Maximum_Iterations | 2000 | Positive, <=INT32_MAX |
| Reprojection_Threshold_Pixels | 3.0 | Finite, >0; conversion must remain positive finite Float32 |
| Confidence | 0.995 | Finite, strictly 0<confidence<1 |

The threshold is **Euclidean forward reprojection error in destination-image
pixels**, not source pixels or a symmetric transfer distance. Confidence is a
RANSAC control, not a posterior probability that registration is correct.

Native inputs are Nx1 CV_64FC2 Core Mats, borrowed only for the callback. Valid
noncontinuous inputs are cloned before native entry to avoid upstream continuity
requirements. No Core handle is retained. Native estimation uses exactly the
legacy overload `findHomography(source,destination,cv::RANSAC,threshold,
native_mask,maximum_iterations,confidence)`. All three reviewed legacy paths
convert coordinates to **Float32 internally**, so Float64 callers do not obtain
Float64 estimator precision. No USAC, RHO, LMEDS or estimator-selection API.

## Stable FINAL-model inliers — not the native mask

**OpenCV 4.1/4.10 return the earlier RANSAC mask after final model refinement;
OpenCV 5.0 recomputes its mask after refinement. The binding NEVER publishes the
upstream mask as its public truth.** See [source review](homography-source-review.md).

For every original caller Float64 correspondence, the binding independently maps
the source through **final H**. Zero denominator or any nonfinite projective
intermediate/result is an outlier. Otherwise it computes
`std::hypot(xp-destination.x, yp-destination.y)`. Inclusion is exactly finite
mapping **and error <= threshold**. No naive huge-value squaring, epsilon cutoff,
or clipping. The classifier is tested independently of OpenCV, including exact
boundary, large hypot, tiny denominator, infinity and scale cases.

Classification walks original correspondence order. Public `Inlier`/`Inliers`
are one-based **correspondence positions**, independent of Ada input bounds,
valid, unique and strictly ascending. They are not array subscripts for an input
whose lower bound differs from one. Excluded points are also checked against final
H by the synthetic raw/public tests; passing only the upstream mask unnoticed is
guarded additionally by a source-structure regression test.

`Homography_Estimate` is limited controlled and owns copied Ada matrix/indices,
following `Pose_Estimate`. Overloaded `Found`, `Inlier_Count`, `Inlier`, and
`Inliers` expose no native handles. A successful estimate requires **>=4
final-model inliers**, even if the native call returned a nonempty matrix. Less
support produces `Found=False`, zero count and empty `Inliers` with bounds 1..0.
Empty native H also produces this legitimate no-model state with success status.
`Homography` on that state raises `OpenCV_Error`; out-of-range `Inlier` also raises.

Nonempty native H must have dims=2, rows=3, cols=3, CV_64FC1, nine finite
coefficients and at least one nonzero coefficient. Failure is a native/binding
error, **not** a legitimate no-model state. No normalization is imposed.

## ABI and failures

The private C `opencv_calib3d_homography` has nine named doubles `h00..h22`;
it is not the rotation-matrix record. An options record and distinct opaque
homography result own the temporary native values. Compiler-derived C/Ada size,
alignment, every field position and C-written interchange qualify both records.

Estimator output is null before fallible work. Accessors clear outputs first:
matrix clears all nine doubles, index/count/found clear to zero. Null destruction
is safe. `unique_ptr` prevents partial result publication; all five existing
exception categories are contained by the C barrier. Existing test-only fault
infrastructure and macOS libc++ isolation are reused; no production test controls.

## Example and limitations

The file-free `homography_synthetic` generates 24 deterministic planar/projective
correspondences and corrupts indices 2, 7 and 15. It prints count, found state,
final support, matrix, outlier survival and independent sample mapping error.
Synthetic thresholds are test policy, not navigation accuracy claims.

No fundamental/essential matrix, recoverPose, triangulation, affine estimation,
image warping, Imgproc/Features dependency, geospatial model, calibration/stereo,
fisheye, new PnP method, estimator selection, or generator is added.