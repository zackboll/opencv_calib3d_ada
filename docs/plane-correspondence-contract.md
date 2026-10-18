# Plane-constrained cross-camera correspondence verification (Task 025)

```text
Image A observed pixel
       |
Camera A calibrated ray
       |
Known world-plane intersection
       |
Fitting-stage accepted world point
       |
Camera B positive-depth/reprojection evaluation
       |
Compare against Image B observed pixel
       |
Accepted or rejected plane-constrained match
```

## Supplied geometry, frames and policy

`Verify_Plane_Constrained_Correspondences` takes positionally matched first/second
`Image_Point_Array` values, independent camera intrinsics and five Brown/OpenCV
coefficients for each camera, two externally supplied `World_To_Camera_Pose`
values, one externally supplied infinite Cartesian `World_Plane`, and mandatory
`Plane_Correspondence_Verification_Options`. Nothing is estimated or optimized.

```text
X_camera = R * X_world + t
C_world  = -R^T * t
```

Both poses and the plane must share a Cartesian frame and length units. Translation
is t, **not** camera world position. The plane may be horizontal, vertical or
arbitrarily tilted; there are no patch boundaries or triangle containment tests.
Exactly coincident physical camera centers are valid. No Essential matrix or
nonzero stereo baseline is required.

`Options.Intersection` is exactly the existing Task 024
`World_Plane_Intersection_Options`, with its incidence, forward-range and
represented-plane-residual validation, availability, order and inclusive gates.
See [the authoritative source contract](world-plane-intersection-contract.md).
`Maximum_Second_Pixel_Error` must be finite and nonnegative, in **pixels**.
Acceptance compares Euclidean distorted-pixel error **<=** this value directly.
Equality passes; one representable step above fails. No default policy, hidden
threshold, empirical epsilon, normalized Sampson comparison or image-boundary
rejection is introduced. Without supplied dimensions, finite off-image projections
are not rejected merely for being outside an actual sensor's FOV.

## Validation, batches and source/target isolation

Before any native entry, validate matched lengths, INT32-representable count,
every coordinate of both pixel arrays, both calibrations (finite positive focal
lengths and finite principal points), all ten Brown coefficients, all twelve pose
components, plane anchor/nonzero finite normal, every intersection policy field,
and the second-pixel threshold. Malformed requests raise `OpenCV_Error` even when
empty. Valid empty input returns `1 .. 0` without native undistortion, Rodrigues,
camera-center or projection calls.

Call `Unproject_Pixels_Onto_World_Plane` exactly once for the entire source array.
Preserve every owned source result, including all diagnostics of rejections.
Do not rebuild world rays, plane arithmetic or inverse Brown distortion. Only
`Accepted(Source)` permits reading its `Accepted_World_Position`. Parallel,
coplanar, at-origin, behind, grazing, out-of-range, nonrepresentable and excessive
represented-residual results never supply coordinates to Camera B.

Compact accepted source positions, matching observed B pixels and original
one-based positions in their original order. The map is **compact position ->
original correspondence position**, not input Ada subscript. Different lower
bounds and multiple elements ending at `Positive'Last` are supported with
subtraction-before-addition indexing. No survivors skips the target evaluator.

For a nonempty compact set, call `Evaluate_AP3P_References` exactly once with
`[(Pose => Second_Pose)]` and B's independent calibration/distortion. This is a
reference evaluation, **not AP3P solving**. Its existing batched implementation
obtains target Rodrigues once, checks all finite camera coordinates, and projects
only strictly positive-depth candidates in one native batch (or no batch if none
have positive depth). It uses native Brown projection, not an Ada reimplementation.
See [the reference contract](ap3p-reference-contract.md).

Camera A supplies the geometric world-point candidate. Camera B supplies the
verification observation and never moves, refines or replaces that candidate.
Changing B's observed pixels cannot change source geometry. A B rejection causes
no alternate intersection, plane adjustment, solver or retry.

## Owned results and guarded diagnostics

Results are private ordinary Ada values: no native handles, pointers, caller-array
references or finalization obligation. Return bounds are `1 .. N` or `1 .. 0`.

| Final disposition | Meaning |
|---|---|
| `Source_Intersection_Rejected` | Task 024 source intersection did not pass. |
| `Second_Non_Positive_Depth` | Finite B depth <= 0; no projected error. |
| `Second_Numerically_Unavailable` | Existing evaluator's per-point arithmetic could not represent the transform or residual. |
| `Second_Reprojection_Rejected` | Positive-depth finite Euclidean pixel error exceeds caller limit. |
| `Plane_Correspondence_Accepted` | Both source policy and B verification passed. |

`Original_Position`, `Disposition`, `Source_Intersection_Evaluated`,
`Second_View_Evaluated` and `Accepted` are always available.
`Source_Intersection` requires its availability flag and returns the **complete
owned** Task 024 value. Its own guarded accessors remain authoritative: a grazing
rejection, for example, still has its computed incidence and signed distance.
Source-only accepted geometry remains diagnostic after B rejects; it is not final
match acceptance. The default private result has no evaluated source or target.

`Second_View_Disposition` requires target evaluation. `Second_Camera_Depth`
additionally requires a non-numerically-unavailable result, preserving meaningful
zero/negative depth. `Second_Pixel_Error` requires `Projected_Positive_Depth`.
Unavailable diagnostics raise `OpenCV_Error`, never publish placeholder zeros.
`Accepted_World_Position` requires **final two-camera acceptance**. It cannot
publish a rejected source or source-only accepted candidate.

## Errors and atomicity

One source rejection or per-point Ada arithmetic unavailability does not reject
unrelated matches. An unavailable error is not a zero-error match. Nonpositive
depth cannot pass regardless of any hypothetical projected residual.

Native errors in source undistortion, Rodrigues, camera-center calculation or
target projection propagate as `OpenCV_Error` with no partial returned array.
Previously assigned complete results remain unchanged if the function assignment
fails. In particular a native projection batch exception is **all-or-error**;
this wrapper does not isolate it to one point or translate it to per-point
`Second_Numerically_Unavailable`. No exception handler surrounds these calls.
No new native operation, C++ code, dependency or private ABI operation is added;
the ABI remains 48 operations. Core retains ownership of temporary Mats and
callback-scoped native borrowing through the reused APIs.

## Accepted-landmark adapter

`Plane_Constrained_Landmark_Observations (Landmarks, Pixels)` returns the existing
owned `Accepted_Landmark_Set`. Supply corresponding observations from a separately
obtained later image. Counts must match; **every** new pixel is validated, including
pixels paired with rejected results. Only `Accepted=True` permits coordinates
through the final guarded accessor. Compact world/pixel/position arrays retain
`Original_Position` stored in each result, even after slicing/rebasing. No final
accepted matches returns `1 .. 0` arrays. No subsequent camera pose is estimated.

Landmarks constrained to a single plane do not themselves establish good 3-D
depth diversity or guarantee robust six-degree-of-freedom relocalization. The
adapter is not evidence that a planar relocalization problem is well conditioned.

## Limits of evidence

Residuals are compatibility checks, not ground-truth proof. Shared errors in
calibration, supplied poses, plane or matching may remain. A planar scene admits
ambiguities and is insufficient evidence of global localization. Near-parallel
intersection is ill-conditioned; iterative undistortion is approximate; finite
Float64 intermediate requirements and large-origin rounding still apply. Use
suitable local coordinates and explicit application policy. Qualified moderate
distortion tolerances are fixture tolerances, not universal accuracy bounds.

No source-to-target homography estimation, image warp, rectification, texture
sampling, feature matching, DTED/DEM lookup, Earth model, geodesy, SLAM, bundle
adjustment or navigation fusion is included. The caller supplies established
corresponding observations. Synthetic examples establish no physical
terrain-registration accuracy.