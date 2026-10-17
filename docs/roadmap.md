# Roadmap

## Task 017 approved scope

Reference-validated AP3P pose refinement only. The operation reuses AP3P,
Task 016 selection, native iterative refinement, and native projection.
Held-out acceptance is caller-controlled. No new optimizer, C ABI operation,
covariance, or geographic frame is added. See
[contract](ap3p-refinement-contract.md).

## Task 016 approved scope

Reference-qualified AP3P pose selection only. The caller supplies every
numerical threshold. Selection uses additional independent references and the
existing evaluator; it does not add a solver, C ABI operation, covariance, or
geographic frame. See [contract](ap3p-selection-contract.md).

## Task 015 approved scope

Guarded native `cv::correctMatches` epipolar correspondence correction only,
with caller-configured conditioning/quality/residual/bound policy and an
all-or-error result. No F estimation, RANSAC changes, correspondence selection,
undistortion, triangulation changes, pose, metric scale or navigation. The earlier
research disposition `SAFETY_GATE_NOT_ESTABLISHED` for universal optimality
Task 015 added guarded native epipolar correction. Its research disposition
`SAFETY_GATE_NOT_ESTABLISHED` for universal optimality stands. Task 016 is the
AP3P selection scope above and does not reopen that correction contract.

## Task 013 approved scope

Independent-reference diagnostics for existing AP3P hypotheses only.
Native projection and distortion stay in `Project_Points`. Ada classifies
camera depth and aggregates projected residuals. No automatic pose
selection, RANSAC, refinement, covariance, or geographic frame.

## Task 012 approved scope

Minimal AP3P camera-pose hypotheses only. Exactly three correspondences,
fixed `SOLVEPNP_AP3P`, all native candidates retained. No automatic pose
choice, fourth-point scoring API, RANSAC, refinement, or geographic frame.

## Task 011 approved scope

Calibrated homography plane-visibility assessment only, preserving all original
hypotheses with explicit dispositions. Compact selected-reference filtering and
pure/near-rotation special handling do not select a physical pose, prove full
cheirality, recover metric scale or add navigation. See
[contract](planar-visibility-contract.md). Task 010's decomposition is unchanged.

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

Task 014 adds native `cv::composeRT` pose composition only: no Jacobians, SLAM, geolocation or navigation fusion. See [contract](pose-composition-contract.md).

Task 018 adds pose inversion, relative camera-to-camera motion, and baseline diagnostics on top of that composition. It does not add an estimator, covariance, or geographic conversion. See [contract](relative-camera-motion-contract.md).
