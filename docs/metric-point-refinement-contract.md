# Fixed-pose metric world-point refinement (Task 020)

`Refine_Metric_World_Points` owns the complete reconstruction/refinement chain.
Callers supply matched distorted pixels, each camera's own intrinsics and five
Brown coefficients, two known world-to-camera poses, all four mandatory
`Metric_Triangulation_Options` thresholds, bounded point-refinement options, and
two mandatory finite nonnegative pixel-worsening allowances. No external source
array, new optimizer, native operation, dependency, or C++ change is involved.
The private ABI remains 48 operations.

## Frames and units

Both poses must already share the caller's Cartesian frame and length units:

```text
X_first  = R1 * X_world + t1
X_second = R2 * X_world + t2
R_relative = R2 * R1^T
t_relative = t2 - R_relative * t1
t_hat = normalize(t_relative)
C1 = -R1^T*t1; C2 = -R2^T*t2; B = norm(C2-C1)

Unit-baseline first-camera coordinates
          |
    Point refinement
          |
    Known baseline scaling (X_first_metric = B * X_first_refined_unit)
          |
Metric first-camera coordinates
          |
    World-frame transform (X_world = R1^T * (X_first_metric - t1))
          |
Refined world-frame 3D point
```

Translation is not camera position. `t_hat` is in the second-camera frame, not
world displacement. Pose, physical baseline, intrinsics and distortion are held
fixed. Length units are meters only if the caller's frame uses meters. Normalized
residuals/RMS are dimensionless; pixel residuals/allowances are in pixels.

## Initial reconstruction and eligibility

Invoke `Triangulate_World_Points` exactly once on the initial batch. Its complete
per-pair value is preserved by `Original_Reconstruction`, including original
source status/quality, geometry, pixel diagnostics, and initial disposition.
An initially `Metric_Policy_Rejected` point is eligible if source status is
`Usable` and both metric geometry and pixel diagnostics are available. Initial
policy rejection is not failed reconstruction, and final acceptance is never
inferred from optimization alone.

Unavailable initial data is explicitly classified as `Initial_No_Baseline`,
`Initial_Source_Unusable`, `Initial_Geometry_Unavailable`, or
`Initial_Pixels_Unavailable`. `Normalized_Refinement.Available` is false in
those cases. No zero diagnostic is interpreted as successful geometry.

Eligible source values enter one `Refine_Triangulated_Points` batch in the
first-camera unit-baseline gauge. Ineligible positions use discriminated
non-Usable sentinels internally; they do not enter optimization and never
replace the preserved original source. Observations are independently normalized
using each camera's calibration. Existing fixed-pose bounds, damping, strictly
decreasing accepted norm, positive depths, and failed-trial nonpublication are
unchanged. No additional convergence epsilon exists.

## Results and final acceptance

Each private `Metric_Point_Refinement_Result` owns its data; outputs are `1..N`
in positional order regardless of input lower bounds, including `Positive'Last`.
Valid empty input returns `1..0` after validation without native calls.

`Normalized_Refinement` exposes the existing outcome, accepted step count,
improved flag (`Outcome=Improved`), initial/final normalized RMS and
`Residuals_Evaluated` guard. `Refined_Diagnostics` contains candidate source,
geometry, and pixels, each guarded by availability. These diagnostics are **not**
an accepted point. `Accepted_World_Position` raises `OpenCV_Error` unless
`Accepted=True` and `Disposition=Refined_Metric_Accepted`. Candidate policy
rejections are labeled `Metric_Policy_Rejected` even when rejection is solely
due to a pixel-worsening allowance.

| Disposition | Meaning |
|---|---|
| Initial reasons above | Reconstruction cannot enter refinement. |
| `Refinement_Low_Parallax` | Existing optimizer's caller-selected acute parallax screen skipped it. |
| `Refinement_No_Improving_Step` | Initial metric reconstruction retained, no accepted optimizer step. |
| `Refinement_Numerically_Unavailable` | Optimizer evaluation/solve unavailable; no candidate published. |
| `Refined_Conversion_Unavailable` | Improved unit point cannot be safely represented/diagnosed in metric/world coordinates. |
| `Refined_Pixels_Unavailable` | Metric candidate exists but finite pixel residuals unavailable. |
| `Refined_Policy_Rejected` | Improved, evaluated candidate fails at least one final quality/allowance check. |
| `Refined_Metric_Accepted` | Improved candidate passes every final check. |

Shared Task 019 conversion checks scale all components and both depths by B,
reject nonzero scaled components underflowing to zero, nonfinite intermediates,
overflow, unrepresentable transforms, and nonpositive actual depths of the
representable world point in either camera. Native rotations are obtained outside
arithmetic handlers. Recompute normalized per-camera errors of the actual world
point after conversion (large origins may round away offsets), and re-assess the
existing acute bearing-line parallax. Native `Project_Points` uses each camera's
own calibration and distortion. Compute finite scaled Euclidean pixel errors.

Acceptance requires `Improved`, representable geometry, positive actual depths,
acute parallax >= caller minimum, both normalized errors <= caller maximum,
both pixel errors <= their respective caller maxima, and BOTH worsening checks:

```ada
if Refined_Error <= Initial_Error then
   return True;
end if;
return Refined_Error - Initial_Error <= Allowance;
```

Direct subtraction of finite nonnegative errors is intentional. Equality passes;
there is no hidden epsilon, clamping, scaled-subtraction loss, or threshold
normalization. An improved normalized RMS can worsen either pixel error.

All inputs/options are validated, even for empty input or zero baseline. Native
undistortion, triangulation, pose/rotation and projection failures propagate as
`OpenCV_Error`, without returning a partially published batch. Per-point Ada
arithmetic failures do not corrupt another point. Core owns Mats; native handles
remain callback scoped and no public raw pointer is added.

## Limitations

The same correspondences fit and diagnose these points: diagnostics are not
statistically independent validation. Neither improvement nor pixel acceptance
proves correct correspondence or better world-coordinate accuracy. Undistortion
is approximate; near-parallel bearings are ill-conditioned; tiny nonzero baseline
is never collapsed with an invented epsilon. Large origins can erase offsets;
overflowing intermediates are rejected even if later cancellation might help.
Use a suitable local Cartesian frame and caller policy. No universal accuracy
bound, covariance, depth uncertainty, bundle adjustment, pose optimization,
DTED/geographic conversion, or navigation fusion is supplied.
