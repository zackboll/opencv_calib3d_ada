# Roadmap

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
Triangulation remains a later separate task.
