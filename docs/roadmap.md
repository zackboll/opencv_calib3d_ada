# Roadmap

## Task 010 approved scope

Calibrated homography motion decomposition only: all native hypotheses,
shared calibration, translation over unknown plane distance, pure rotation,
signed projective-scale handling, fixed C/Ada storage, and independent oracles.
No visibility filtering, automatic pose selection, metric translation,
different-intrinsics overload, or Task 011 work is included.

Task 008 adds pure-Ada triangulation parallax and caller-selected quality
diagnostics, without changing reconstruction or Usable status. See
[contract](triangulation-quality-contract.md) and
[qualification](triangulation-quality-validation.md). Refinement, covariance,
calibration and navigation/geospatial integration remain outside this task.

Task 007 normalized two-view triangulation adds standalone reconstruction and
qualification. No metric scale, refinement, rectification, navigation
or geospatial integration is included. See [progress](triangulation-validation.md).

1. Validate/bootstrap robust PnP (`Project_Points`, `Camera_Center`, EPNP RANSAC).
2. Installed/clean-consumer relocation and linkage qualification.
3. Add pose refinement only after establishing a cross-version policy (4.1 lacks
   public `solvePnPRefineLM`).
4. Add reprojection diagnostics and correspondence convenience APIs.
5. Keep geospatial/DTED conversion in a separate layer that defines a local metric
   world frame.
6. Integrate Features correspondences at an application/navigation layer rather
   than creating a hard dependency from Calib3D to Features.


Task 005: robust fundamental-matrix epipolar verification only (see its contract).
Task 006: Essential matrix and relative camera pose; not part of Task 005.
Standalone normalized triangulation was added in Task 007.

Task 009 adds bounded normalized point-only refinement at fixed relative pose.
See [contract](triangulation-refinement-contract.md) and
[qualification](triangulation-refinement-validation.md). It does not add bundle
adjustment, metric scale, covariance, depth uncertainty or navigation conversion.
