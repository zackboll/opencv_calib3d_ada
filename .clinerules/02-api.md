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

## Subsequent approved feature tasks

The bootstrap API restrictions above apply specifically to Task 001's
initial bootstrap qualification. They are not permanent restrictions
against expanding OpenCV.Calib3D.

Task 002 and subsequent approved feature tasks may extend the public
Ada API when explicitly authorized by the repository owner.

The scope of each subsequent task is defined by its approved task
instructions. A task may introduce additional bindings, including
homography, triangulation, refinement, or other Calib3D operations,
when that specific functionality is authorized.

Such authorization does not permit unrelated feature expansion.

All subsequent tasks must preserve the permanent architectural rules:

- Handwritten thick Ada bindings; no production binding generators.
- OpenCV.Core ownership of application Mat objects.
- Callback-scoped native Mat borrowing through Module_Interop.
- No public C++/STL types or raw native pointers.
- Stable, qualified C/Ada ABI layouts.
- Native exception containment and failure atomicity.
- Correct coordinate-frame and numerical contracts.
- OpenCV 4.1, 4.10, and 5.0 compatibility qualification.
- Independent mathematical oracles and full regression testing.
- Existing sanitizer and CI review gates.
- Windows qualification after merge only.
- No automatic merging, releases, tags, or version bumps.

The historical Task 001 exclusions must not be interpreted as
prohibiting subsequently authorized feature tasks.

Task 010 is explicitly authorized to add calibrated homography motion
decomposition only, subject to its own numerical, ABI, source-review,
testing, and compatibility requirements.
