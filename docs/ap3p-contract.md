# Minimal AP3P pose hypotheses

`Solve_AP3P_Hypotheses` binds `cv::solveP3P` with fixed `SOLVEPNP_AP3P`.
It returns every validated native world-to-camera hypothesis. It does not
choose one pose, run RANSAC, refine, or rank candidates.

```text
exactly 3 world points + 3 pixels + K + five distortion coefficients
    -> 0 .. 4 World_To_Camera_Pose hypotheses
```

Image points are pixels. The wrapper passes those pixels, K, and distortion
into `solveP3P`. Native code undistorts before the minimal solver. Ada does
not undistort first.

## Counts

OpenCV 4.1 requires exactly three points. OpenCV 4.10 and 5.0 also accept
four, but this binding rejects every count other than three before native
entry. A real native zero-solution result is success and returns `1 .. 0`.
A malformed native result or C++ failure raises `OpenCV_Error`.

Native order is not a contract. OpenCV 4.1 keeps solver order. OpenCV 4.10
and 5.0 may sort candidates by reprojection residual. Do not compare
Rodrigues coefficients or candidate indices across versions.

## Frame and scale

`X_camera = R * X_world + t`. `Translation` is `t`, not the camera position.
`Camera_Center` is `C_world = -R^T * t`. Translation uses the units of the
world points. No geographic frame is inferred. Unlike essential-matrix
recovery, the baseline magnitude is determined by those world coordinates.

## Admissibility

Checked before native entry, using exact zero rather than an invented epsilon:

- three finite non-repeated, non-collinear world points;
- three finite pixel observations;
- positive finite focal lengths and finite principal point and distortion;
- nonzero world baseline and nonzero cross-product area;
- distinct first two undistorted bearings and a third bearing not exactly coplanar with them;
- no intermediate product that overflows Float64.

Bearing checks use the normalized coordinates returned by `undistortPoints`
with the same K and distortion. Those rays are not the original distorted
pixels. The subsequent `solveP3P` call still receives the original pixels.
Iterative undistortion does not restore an exactly collinear ray set: on
OpenCV 4.10 the documented radial fixture recovers the ideal line only to
about `1.4e-5`, and that nonzero result remains admissible. A portable exact
rejection is a bearing set whose normalized `y` components are all zero, so
the third bearing lies in the plane of the first two with a bitwise-zero
product. The ideal `y=0.5` line is mathematically collinear, but unit
normalization through `sqrt` is not a portable bitwise zero and is not used
as that gate. No universal epsilon is applied to the residual.

Repeated points, collinear points, identical bearings, and coplanar
three-ray geometry are invalid. They are not reported as an empty success.
A finite admissible configuration for which AP3P returns zero solutions is
an empty success.

## ABI

One new private export, `opencv_calib3d_solve_ap3p_hypotheses`, uses the
existing six-double pose record inside a caller-owned aggregate:

```c
typedef struct opencv_calib3d_ap3p_hypotheses {
    int32_t count;
    opencv_calib3d_pose candidates[4];
} opencv_calib3d_ap3p_hypotheses;
```

The result is zeroed on entry and replaced only after every candidate has a
finite Rodrigues vector, finite translation, and a proper rotation matrix.
Failure clears the whole aggregate. No C++ exception crosses the C ABI.
The private ABI is 46 matching imports and exports.
