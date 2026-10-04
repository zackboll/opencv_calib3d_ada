# Coverage

Bootstrap public coverage:

- camera intrinsics (`fx`,`fy`,`cx`,`cy`, zero skew);
- five-coefficient Brown/OpenCV distortion model;
- world-to-camera Rodrigues pose value;
- camera-center conversion;
- sparse `projectPoints` without Jacobian;
- fixed-EPNP `solvePnPRansac` without extrinsic guess;
- explicit found/no-pose state;
- one-based sorted inlier indices.

Bootstrap AUnit source registers 10 cases covering projection oracles, distortion,
camera-center frame semantics, clean/outlier synthetic PnP, validation and ABI
layout. These cases are **present but not claimed passing** until native bootstrap
qualification is run.

Out of scope: broad calibration, stereo, homography, essential/fundamental,
triangulation, arbitrary SolvePnP methods, refinement, fisheye, undistortion,
USAC configuration, Features integration and geographic/DTED semantics.
