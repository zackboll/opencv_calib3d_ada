# Triangulation parallax and quality contract

`Measure_Stereo_Parallax` is pure Ada and may run before triangulation. Inputs
are calibrated normalized pinhole observations, not distorted pixels.
For `X_second = R X_first + lambda t_hat`, form

```text
b1 = normalize(x1,y1,1)
b2_first = normalize(R^T normalize(x2,y2,1))
d = b1 dot b2_first
c = norm(b1 cross b2_first)
theta = atan2(c,d)       forward directed ray angle, 0 .. pi
alpha = atan2(c,abs(d))  acute line-intersection angle, 0 .. pi/2
```

The transpose is essential. Translation is never added to a bearing and its
magnitude has no effect. Antiparallel rays have theta=pi but alpha=0: like
parallel rays, their underlying lines have weak angular intersection geometry.
The optional conditioning threshold therefore uses alpha, not theta.

Bearings use the existing scaled Float64 `Unit_Vector`; the transformed second
bearing is normalized again. The cross norm is also scaled to avoid squaring
tiny values directly. Two-argument Ada arctangent preserves small angles that
`acos(dot)` can lose. No arbitrary angle epsilon or clamping is applied.
Finite observations and a finite nonzero translation are required. Shared
Task 007 rotation validation requires R^T R approximately I and det(R)
approximately +1 with absolute tolerance 1e-9, without rotation repair.
Nonfinite input/arithmetic raises `OpenCV_Error`; no `Constraint_Error` escapes.

`Assess_Triangulation` measures these angles and reads, but never modifies or
recreates, caller-supplied `Triangulated_Point` values. All three arrays must
have equal counts; pairing is by iteration order, independent of lower bounds.
Both APIs return one-based arrays, including `1 .. 0` for empty input after
validation. Non-Usable results retain their angular information without
accessing absent discriminated fields.

Caller-selected `Triangulation_Quality_Options`:

* `Minimum_Acute_Parallax_Radians`: finite, 0 .. pi/2, default 0 (disabled).
* `Maximum_Normalized_Reprojection_Error`: finite, nonnegative, default
  Float64'Last (disabled). Residuals are dimensionless, not pixels.

Comparisons are inclusive with no hidden epsilon: alpha >= minimum; both
residuals <= maximum. `Passes_Reprojection_Limit` is false for non-Usable
results. `Accepted` is true only when original status is Usable and both
flags pass. Allegedly Usable public values must have finite position, finite
positive depths and finite nonnegative errors, or assessment raises
`OpenCV_Error`. No recomputation or physical-consistency test is performed.

There is no universal acceptable parallax cutoff. A numerically usable 3-D
point can still have poor triangulation geometry. Screening is caller policy:
`Triangulated_Point.Status` and `Triangulate_Normalized` remain unchanged.
Coordinates remain in unit-baseline units; metric baseline magnitude is unknown.
Parallax is not covariance, range variance, navigation error, confidence,
accuracy probability or a guaranteed depth-error bound. Statistical uncertainty
requires measurement-noise, calibration, pose and baseline assumptions not
provided here. No automatic rejection, refinement or correspondence removal
is performed. Native C ABI remains 43 exports/imports; no new native call,
handle, dependency or fault checkpoint is added.

## Optional subsequent point refinement

Task 009's [point-refinement API](triangulation-refinement-contract.md) is separate
from this unchanged assessment API. It reuses acute parallax as caller policy,
with zero default minimum, and never changes input statuses or arrays. Improved
coordinates still have unknown metric scale and no covariance/depth guarantee.
