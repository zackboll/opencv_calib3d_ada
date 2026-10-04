# API policy

The bootstrap implements only a narrow common pose slice:
- camera intrinsics;
- five Brown/OpenCV distortion coefficients;
- `Project_Points`;
- fixed-EPNP `Solve_PnP_RANSAC`;
- inlier indices;
- world-to-camera pose and camera-center conversion.

Do not broaden into calibration, stereo, homography, USAC, pose refinement,
fisheye, DTED, feature matching, or estimator policy in bootstrap qualification.
