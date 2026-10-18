# Epipolar-gated three-view metric landmark qualification (Task 023)

`Qualify_Three_View_Landmarks` consumes three positionally matched pixel arrays,
three independent calibrations (intrinsics and five Brown coefficients each),
three **supplied** world-to-camera poses, and mandatory caller policy. It returns
one owned private Ada result per original track, including rejected tracks.
No pose is estimated or changed; no solver, optimizer, distortion implementation,
native operation, dependency, or C ABI change is introduced (48 operations).

## Frames, units and validation

All poses must share the caller's Cartesian world frame and length units:

```text
X_camera = R * X_world + t
C_world = -R^T * t
```

Translation is t, **not camera position**. Coordinates use the supplied physical
stereo baseline, not Essential-matrix scale. World units are meters only if the
caller uses meters. Sampson and normalized fitting errors are dimensionless;
fitting/third-view pixel errors and worsening allowances are pixels.

Before any native entry or early return, validate every observation, calibration,
pose, count, and policy field, including unused refinement settings in
`Initial_Only`. Counts must match and fit INT32. Pixels, pose components, principal
points and all Brown coefficients must be finite; focal lengths finite positive.
Every numerical acceptance limit must be finite nonnegative. Existing metric
and refinement angle bounds are 0..pi/2; refinement iterations are 1..100.
No empirical threshold defaults are added. All acceptance comparisons are direct
and inclusive, with **no hidden epsilon**. Valid empty requests return `1..0`
without native entry.

## Data flow and first–second screening

```text
First and second poses/calibrations/pixels ONLY
  -> Verify_Known_Pose_Epipolar
  -> individual Correspondence_Errors
  -> compact passing pairs and original-position mapping
  -> Task 019 OR Task 020
  -> fitting-stage accepted candidate selection (final, no retry)
                                                |
Third supplied pose/calibration/pixels ----------+
  -> Evaluate_AP3P_References, one existing-pose hypothesis, batched points
  -> per-track positive-depth and third-pixel policy
  -> guarded accepted world coordinates
```

The Task 022 call uses `Minimum_Correspondences => 1`, the caller's
`Maximum_Normalized_Sampson_Error`, and finite `Float64_Value'Last` for its RMS
limit. This deliberately disables **whole-set RMS rejection**, not an independent
RMS acceptance policy. The whole Task 022 result need **not** be accepted. One
undefined or mismatched pair cannot disqualify other individually passing pairs.
There are no three-view whole-set acceptance statistics.

Exact zero physical source baseline yields `No_Stereo_Baseline` for every track,
with no E, Sampson metric, triangulation, or fabricated point. Tiny representable
nonzero baseline is never silently treated as zero. Undefined individual Sampson
metrics yield `Epipolar_Undefined`; defined values above the caller limit yield
`Epipolar_Rejected`. Defined metrics and their inclusive threshold flags survive.
`Epipolar_Disposition` separately reports not evaluated, undefined, rejected, or
passed, so later rejections do not erase screening provenance.

Only passing pairs enter owned compact first/second arrays. The mapping is
`compacted position -> original one-based correspondence position`, never source
Ada subscripts. All three input arrays may have unrelated lower bounds, including
singleton or multiple elements ending at `Positive'Last`. Positional arithmetic
is `First + (Position - 1)`, subtracting before adding. No survivors means no
triangulation; third observations have nevertheless already been validated.

## Reconstruction, refinement and source selection

* `Initial_Only`: call `Triangulate_World_Points` once on the compact batch.
  Select only `Metric_Accepted` with available world geometry.
* `Prefer_Refined_With_Original_Fallback`: call `Refine_Metric_World_Points` once;
  obtain initial values using `Original_Reconstruction` (no extra triangulation).
  Select an accepted Task 020 refined point first. Otherwise select the original
  **only** when independently `Metric_Accepted` with available geometry.
* `Require_Accepted_Refinement`: select only accepted Task 020 refined coordinates.
  An accepted original without accepted improving refinement is unavailable.

Task 020 retains its original fixed-pose, first-camera unit-baseline optimization
and subsequent metric conversion and fitting-policy checks. A rejected optimizer
candidate never supplies accepted coordinates. `Candidate_Source` is explicitly
none, initial, or refined. Selection is complete before third pixels are read
for evaluation. **The third observation never enters Essential screening,
triangulation, refinement, or candidate selection.** After third rejection there
is no retry with another candidate. Third pixels cannot change reconstructed XYZ.

## Third-camera held-out verification

Compact selected world candidates, corresponding third pixels, and a second
owned original-position mapping. Call `Evaluate_AP3P_References` once with
`[(Pose => Third_Pose)]`, using the third camera's own intrinsics and distortion.
This neither solves AP3P nor selects a hypothesis. Its existing evaluator obtains
native Rodrigues once per hypothesis and checks all three camera coordinates.
As corrected by Task 023-R1, it compacts finite positive-depth references with
saved depths and an owned original-position map, then calls native `Project_Points`
**once per hypothesis with eligible references**, outside both per-reference
loops. No eligible references means no native projection. Here the single supplied
third-pose hypothesis means zero or one native batch, distinct from the one call
to the Ada evaluator. The reviewed original head called the Ada evaluator once
but still projected each eligible reference separately; that was not batching.

OpenCV batch/vectorized paths may round differently from singleton calls.
No cross-version or batch-size bitwise equality is promised. Units, geometry,
inclusive policy comparisons and failure semantics are preserved, without a new
epsilon; points exactly at a policy boundary may reflect native rounding changes.

* `Projected_Positive_Depth`: finite positive depth and finite nonnegative pixel
  error. Error <= `Maximum_Third_Pixel_Error` passes, including equality.
* `Non_Positive_Depth`: zero or negative depth rejects, no valid pixel error.
* `Numerically_Unavailable`: per-point camera arithmetic or pixel residual cannot
  be represented; rejects, no valid depth/error exposed by this operation.

Native Rodrigues/projection exceptions and invalid native outputs propagate as
`OpenCV_Error`; they are **not** routine per-point numerical rejection. The
complete operation returns a fully constructed owned result or propagates an
exception, without partial publication. Temporary Mats remain Core-owned with
callback-scoped borrowing. No handles, pointers, or borrowed memory escape.

## Result dispositions and accessors

| Final disposition | Meaning |
|---|---|
| `No_Stereo_Baseline` | Exactly zero physical first–second baseline; no geometry fabricated. |
| `Epipolar_Undefined` | Sampson metric unavailable. |
| `Epipolar_Rejected` | Defined Sampson error exceeds caller limit. |
| `Stereo_Source_Unusable` | Initial source/conversion cannot supply an accepted metric point. |
| `Stereo_Quality_Rejected` | Initial metric geometry exists but its fitting policy rejects it. |
| `Refinement_Required_Unavailable` | Required-refinement mode has no Task 020 accepted point. |
| `Third_Non_Positive_Depth` | Selected candidate has zero/negative third depth. |
| `Third_Numerically_Unavailable` | Third evaluator reports per-point arithmetic unavailable. |
| `Third_Reprojection_Rejected` | Valid third error exceeds caller limit. |
| `Three_View_Accepted` | Every applicable fitting, screening and held-out check passed. |

Required-refinement unavailability takes precedence over initial-source/quality
labels in that mode; detailed Task 019 and Task 020 dispositions remain available.

`Original_Position`, `Disposition`, `Epipolar_Disposition`, discriminated
`Epipolar_Error`, `Refinement_Requested`, availability predicates, `Candidate_Source`
and `Accepted` are always available. `Has_Initial_Reconstruction` guards
`Original_Reconstruction`; `Has_Refinement_Diagnostics` guards
`Refinement_Diagnostics` (the entire owned Task 020 value with existing accessors).
Requesting an absent diagnostic raises `OpenCV_Error`.

`Selected_World_Position` requires a selected fitting-stage-approved candidate.
It is diagnostic and **not** final three-view acceptance. `Third_View_Evaluated`
guards `Third_View_Disposition`; `Third_Camera_Depth` requires a non-numerically-
unavailable evaluation, retaining meaningful zero/negative depth. `Third_Pixel_Error`
requires `Projected_Positive_Depth`. Only `Three_View_Accepted` permits
`Accepted_World_Position`. Missing geometry/residuals are never exposed as numeric
zeros. Detailed source and optimizer numbers are reused, not duplicated.

## Compact adapter and subsequent observations

`Three_View_Landmark_Observations (Results, Pixels)` returns the existing owned
`Accepted_Landmark_Set`. The distinct name preserves source compatibility of
unqualified aggregates passed to the Task 020 adapter. Counts must match; every
new pixel is validated, even for rejected tracks. It reads coordinates only via
`Accepted_World_Position` after `Accepted=True`, compacts `World_Points` and
`Image_Points`, and preserves `Original_Positions` stored in each result, including
when callers slice/rebase results. No accepted tracks returns `1..0` arrays.

Pixels can be observations in a subsequent fourth image. This adapter does not
estimate or verify that camera. Callers may pass compact arrays to
`Relocalize_Camera` with separately selected held-out landmarks. Diagnostic initial
points or unaccepted optimizer candidates are never substituted.

## Limits of geometric evidence

A third observation adds geometric evidence but does **not** establish true
statistical independence, correct feature identity, global localization, physical
ground truth, or a quantitative uncertainty bound. Calibration, supplied poses,
matching and reconstruction can share systematic errors. Epipolar consistency
alone cannot distinguish a wrong finite disparity/depth; even three-view
acceptance means compatibility with supplied inputs and policy, not match truth.

Existing representability limits apply: iterative undistortion is approximate;
near-parallel bearings are ill-conditioned; tiny baseline is not zero; large
world origins can erase offsets; overflowing intermediates reject even if later
cancellation could help. Metric conversion rejects nonzero components that scale
to exact zero. Use a suitable caller-defined local Cartesian frame and policy.
No covariance, Earth/geospatial model, uncertainty propagation, Features, Video,
DTED, coordinate-system conversion, SLAM, bundle adjustment, or navigation fusion
is included in this binding.