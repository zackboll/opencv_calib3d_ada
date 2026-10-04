# Roadmap

1. Validate/bootstrap robust PnP (`Project_Points`, `Camera_Center`, EPNP RANSAC).
2. Installed/clean-consumer relocation and linkage qualification.
3. Add pose refinement only after establishing a cross-version policy (4.1 lacks
   public `solvePnPRefineLM`).
4. Add reprojection diagnostics and correspondence convenience APIs.
5. Keep geospatial/DTED conversion in a separate layer that defines a local metric
   world frame.
6. Integrate Features correspondences at an application/navigation layer rather
   than creating a hard dependency from Calib3D to Features.
