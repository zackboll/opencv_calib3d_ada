# Robust fundamental-matrix contract

`OpenCV.Calib3D` adds only two-view epipolar verification. There is no Essential
matrix, relative pose, triangulation, calibration, rectification, homography
expansion, Features/Imgproc Ada dependency, or public estimator selection.
Essential matrix and relative camera pose are reserved for Task 006.

## Public values and relation

`Fundamental_Matrix` is a distinct 0..2 by 0..2 Float64 array. For homogeneous
image points `p1=(x1,y1,1)`, `p2=(x2,y2,1)`, the relation is `p2^T * F * p1 = 0`.
F has arbitrary nonzero common scale and sign: **F(2,2)=1 is not promised**.
Do not compare estimated coefficients across native versions. F constrains
two-view epipolar geometry; it does not itself recover metric scale, camera
position, baseline, or 3-D terrain location.

`Maximum_Epipolar_Error(F,First_Point,Second_Point)` is pure Ada arithmetic on
caller Float64 values. With `l2=F*p1=(a2,b2,c2)` and
`l1=F^T*p2=(a1,b1,c1)`, its defined result is:

```text
d2 = abs(a2*x2+b2*y2+c2) / hypot(a2,b2)
d1 = abs(a1*x1+b1*y1+c1) / hypot(a1,b1)
Maximum_Error_Pixels = max(d1,d2)
```

This is the square root of the legacy native callback's maximum squared
point-to-line distance, **not Sampson distance**, and not a sum of distances.
All input matrix entries and point coordinates must be finite, or OpenCV_Error
is raised. Zero line normals, nonfinite products or sums (including partial
sums), numerators, norms or final distances return `Defined=False`. Scaled hypot
avoids naive squaring overflow; no arbitrary epsilon, clamping, NaN output or
Constraint_Error escapes. Finite input does not guarantee representable geometry.
Scale invariance is subject to representable intermediate arithmetic/rounding.

## Estimation and options

`Estimate_Fundamental_RANSAC(First_Points,Second_Points,Options)` requires equal
counts **>=15**, fitting signed INT32, with finite coordinates. Ada rejects counts
before creating Core Mats; raw C independently rejects before the native call.
The reviewed implementations use a direct seven-point solve at 7 and silently
select LMeDS at 8..14, even when FM_RANSAC is requested. At 15+ the exact common
call is:

```cpp
cv::findFundamentalMat(first, second, cv::FM_RANSAC,
    options->epipolar_threshold_pixels, options->confidence, native_mask)
```

`Fundamental_RANSAC_Options` contains only `Epipolar_Threshold_Pixels` (default
3.0) and `Confidence` (default 0.99). There is deliberately no Maximum_Iterations:
the legacy common overload has a **fixed native robust-iteration ceiling: 1000**.
Adaptive stopping can use fewer iterations.

The shared private `fundamental_profile.hpp` implements the exact native
arithmetic profile; a private C validation export lets Ada preflight use the same
rule without duplicating float rounding boundaries:

* Threshold is finite and positive. Native `float(double(threshold*threshold))`
  must be finite and positive. Squaring occurs **before** conversion. Binary32
  underflow-to-zero and rounding-overflow-to-infinity are rejected, including
  values such as 1e-30 whose unsquared float conversion would still be positive.
  The helper tests both rounding boundaries and avoids out-of-range C++ casts.
* Confidence is finite and **DBL_EPSILON <= c <= 1-DBL_EPSILON**, inclusive.
  This is narrower than a simple (0,1) contract and prevents OpenCV's silent
  replacement with 0.99. C++ statically qualifies IEEE binary64 with
  `DBL_EPSILON=2^-52` and IEEE binary32; Ada uses that C profile, not an assumption
  about Model_Epsilon.

## Precision, ownership and results

The raw input schema is 2-D Nx1 CV_64FC2. Logically valid noncontinuous Core
Regions are snapshotted before reshape. Core handles are callback-scoped and
never retained. **findFundamentalMat converts input points to CV_32F before the
legacy estimator operates**: native model estimation is not a full Float64
coordinate estimator. Float64 API coordinates support consistent Ada values and
final public classification, not a claim of native double-coordinate precision.

A nonempty native F must be exactly 2-D 3x3 CV_64FC1, with nine finite coefficients
and at least one nonzero. Invalid nonempty output is a native/binding error, not
mathematical no-model. The seven-point solver imposes a zero determinant and the
eight-point solver explicitly zeroes its smallest singular value. The binding
adds no fragile determinant/rank threshold or scale/sign normalization.

`Fundamental_Estimate` is limited/controlled and owns Ada values. `Found`,
`Fundamental`, `Inlier_Count`, `Inlier`, `Inliers` overload the existing result
accessor pattern. Native masks are never public policy. The raw ABI classifies
original Float64 pairs with final F and robust C++ hypot; the Ada wrapper
independently calculates its public classification with Maximum_Epipolar_Error:

```text
inlier iff error.Defined AND error.Maximum_Error_Pixels <= caller threshold
```

Correspondence positions are one-based, valid, unique and strictly ascending,
independent of the caller array lower bounds. Final support must be >=7. Native
empty F or insufficient final support gives Found=False, zero inliers, empty
`1 .. 0` array, and Fundamental raises OpenCV_Error. No-model is not an ABI error.
The raw ABI initializes result=null before fallible work and clears accessor
outputs before validation (flag/count/index zero, all matrix fields zero).
destroy(null) is safe. C++ exceptions never cross the C ABI. Native result
publication uses unique ownership; an Ada result guard destroys native resources
even on translated exceptions.

Backend mapping and Core pin are unchanged. Qualification evidence, rather than
the presence of tests, is recorded in `fundamental-validation.md`.
