# Bounded normalized point refinement

`Refine_Triangulated_Points` is pure Ada, point-only and deterministic for fixed
inputs on a given floating-point implementation. It does not call a native solver.
The original observations, pose and triangulation arrays are never modified.
`Triangulate_Normalized`, its statuses and the quality API are unchanged.

## Geometry and objective

Coordinates stay in the first-camera frame with unit baseline:
`Q = R*X + normalize(t)`, `P1=[I|0]`, `P2=[R|t_hat]`. Translation normalization
preserves sign. Neither camera pose nor baseline gauge is optimized.

The four residuals are predicted minus observed normalized coordinates:
`(X/Z-a1, Y/Z-b1, Qx/Qz-a2, Qy/Qz-b2)`. The objective is unweighted two-camera
least squares. Comparisons use the equivalent robust norm
`E=hypot(hypot(r0,r1),hypot(r2,r3))`; normalized RMS is `E/2`, dimensionless,
not pixels. Input depths/errors are validated but never used as the objective.

The analytic Jacobian rows are `[1/Z,0,-u1/Z]`, `[0,1/Z,-v1/Z]`,
`(R0j-u2*R2j)/Qz`, and `(R1j-v2*R2j)/Qz`. No production finite differences.
All intermediates are finite checked; representability failures are not zero steps.

## Bounded damping

Scale the Jacobian by `S=max(abs(Jij))`, reject zero/unrepresentable S, and solve
`(Js^T Js + lambda I)y = -Js^T r`, `update=y/S`. A checked 3x3 Cholesky solve
requires finite positive pivots. The negative gradient matches the residual sign.

Lambda starts at `1e-3`; each iteration allows eight trials. On rejection it
increases tenfold, capped at `1e12`. After acceptance it decreases tenfold,
floored at `1e-12`. The public iteration budget is 1..100 (default 20).
Stop at the budget, exactly zero initial/current norm, or eight unsuccessful
trials. No convergence or mathematical-optimum guarantee is made.

Only a finite candidate with both actual depths strictly positive and
`new_norm < current_norm` is accepted. No acceptance epsilon permits worsening.
A rejected candidate never changes the accepted state. Final diagnostics are
recomputed from the final geometry, including both depths and per-camera hypot
errors. Only an improved result publishes these diagnostics.

## Validation, policy and dispositions

Equal array lengths are required, paired by iteration order regardless of lower
bounds; output is `1..N`, or `1..0` after validation. The shared pose validator
requires finite SO(3) rotation at tolerance `1e-9` and finite nonzero translation.
All observations must be finite. Usable points require finite XYZ, finite positive
reported depths and finite nonnegative reported errors; nonpositive actual depth
in either camera raises `OpenCV_Error`. Numerical unavailability of otherwise
valid geometry is a per-point result, not batch failure.

The finite caller minimum acute angle must be in `0..pi/2`. Zero disables
screening, with no hidden angle threshold. Positive minima use the existing
`Measure_Stereo_Parallax`, accepting equality. Attempting low-parallax geometry
does not establish reliable range.

| Outcome | Meaning |
|---|---|
| `Skipped_Unusable` | Original status is not Usable; absent fields are not accessed. |
| `Skipped_Low_Parallax` | Acute line angle below the positive caller minimum. |
| `No_Improving_Step` | Valid initial objective, no accepted strictly decreasing update. |
| `Improved` | At least one update accepted; Usable output with fresh diagnostics. |
| `Numerically_Unavailable` | Initial evaluation or initial solver cannot be represented safely. |

Every non-Improved outcome preserves the supplied point value exactly, with zero
accepted steps. RMS fields are meaningful only when `Residuals_Evaluated=True`;
otherwise both are zero. A later numerical trial failure retains earlier accepted
improvements; it never discards them or publishes a failed trial.

This is not bundle adjustment, pose refinement, covariance, depth uncertainty,
metric baseline recovery, geospatial conversion or navigation position. Lower
reprojection residual does not prove correct correspondence or lower 3-D error.