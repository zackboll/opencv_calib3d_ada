# Normalized two-view triangulation

Task 008's separate [quality diagnostics](triangulation-quality-contract.md)
do not change this solver, its Status contract, or any native ABI behavior.
`Usable` means finite positive-depth reconstruction with defined residuals,
not well-conditioned ray geometry. Measure parallax before triangulation or
assess existing values afterward using caller-selected thresholds.

`Triangulate_Normalized` is standalone; it neither estimates Essential matrices
nor performs RANSAC. Applications may select `Pose_Inliers` before calling it.

The gauge is lambda=1 in `X_second = R X_first + lambda t_hat`:
`P1=[I|0]`, `P2=[R|normalize(t)]`, `C2_first=-R^T t_hat`.
Positions are in the **first camera frame in unit-baseline units, not meters**.
Multiply by a known physical baseline B for metric coordinates. No baseline
magnitude is recovered. Translation direction is not camera position.

Equal finite counts fitting INT32 are required. One pair is permitted. Empty
requests return `1 .. 0` after pose validation, without native triangulation.
Each entry of `R^T R-I` and `det(R)-1` must be within absolute Float64 tolerance
1e-9. Rotations are never repaired. Nonzero finite translation is robustly
normalized preserving sign. Output is one-based in pair iteration order,
regardless of caller bounds, without compaction.

Classification order: nonfinite homogeneous component -> Unrepresentable_Point;
exact W=0 -> At_Infinity; otherwise divide regardless of W sign or size.
Nonfinite Euclidean coordinates or second-frame transform -> Unrepresentable_Point.
Either depth <=0 -> Non_Positive_Depth. Nonfinite normalized projection,
residual or robust hypot -> Undefined_Reprojection. Otherwise -> Usable.
Only Usable publishes position, both depths, and dimensionless normalized
Euclidean reprojection errors, **not pixels**. No error rejection threshold,
parallax cutoff or W epsilon is applied. Tiny parallax may produce enormous,
poorly conditioned finite points; Usable is not an uncertainty certificate.

Core inputs are snapshotted before explicit continuous 2xN CV_64FC1 matrices
are populated. Projections are explicit 3x4 CV_64FC1. The complete output must
be 4xN CV_64FC1 before any read. Native schema failure is a batch error;
individual degeneracy is a status. The owned private handle publishes only
after complete classification. Invalid statuses have zero numeric fields.
Accessors clear records before validation; null destruction is safe. Exceptions
stay behind the guarded ABI; public Ada values retain no Core handles.
Raw empty inputs must be typed two-dimensional 0x1 CV_64FC2 Mats, not untyped
empty Mats.