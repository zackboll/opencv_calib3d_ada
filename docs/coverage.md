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

The initial bootstrap registered 10 AUnit cases. Qualification registers 12:
identity/translation/rotation projection, all-five distortion, identity/rotated
camera center, clean/outlier PnP, native false/no-pose behavior, intrinsics,
options/count validation, and compiler-derived C/Ada layout/interchange for every
member of intrinsics/distortion/pose. Local OpenCV 4.10.0: 12 executed, 12 passed,
zero failed assertions and unexpected errors. The runner fails its process on
either kind of AUnit failure.

Python configuration/static suite: 16 tests, including isolated configure backend,
missing header/library, invalid version and CI topology rejection fixtures.
Raw ABI uses real Core-created matrices: null required arguments, depth/channel/
Nx1 schema failures, count mismatch/minimum, invalid camera/options/distortion,
inlier accessor range/null checks, cleared failure outputs and destroy(null).
It exercises robust outliers and native no-pose behavior in production and
fault-injection builds. Exception injection covers all five exception categories
before/after projection, before/after native solve and before publication, plus
camera-center and result-access stages. Projection failure preserves output;
scalar outputs clear and unpublished results are reclaimed.

Both actual-shim Linux ASan/UBSan variants passed locally with leak detection,
halt-on-error and no suppressions; production excludes test-hook symbols.
This does not prove real allocator exhaustion or instrument all dependencies.
See `bootstrap-validation.md` and PR Actions for cross-platform/matrix evidence.

Out of scope: broad calibration, stereo, homography, essential/fundamental,
triangulation, arbitrary SolvePnP methods, refinement, fisheye, undistortion,
USAC configuration, Features integration and geographic/DTED semantics.
