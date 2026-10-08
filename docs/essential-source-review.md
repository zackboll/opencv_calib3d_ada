# Immutable Essential / recoverPose source review

| Version | Official immutable revision | Backend |
|---|---|---|
| 4.1.0 | `371bba8f54560b374fbcd47e7e02f015ac4969ad` | calib3d |
| 4.10.0 | `71d3237a093b60a27601c20e9ee6c3e52154e8b1` | calib3d |
| 5.0.0 | `40738fb16ceddb5fb3fea747585f7ce6abb0605b` | geometry |

Downloaded immutable official headers, `five-point.cpp`, `ptsetreg.cpp` and
`precomp.hpp` for all three revisions. Existing header/registrator/factory
SHA-256 and Git blob SHA-1 values were reverified; the three five-point source
hashes are added separately to `source-provenance.json`. No upstream source is
vendored. OpenCV 4 uses `modules/calib3d`; OpenCV 5 uses `modules/geometry` and
`opencv2/geometry/3d.hpp`. Public Ada remains `OpenCV.Calib3D`.

## API evolution, fixed-1000 profile and normalized units

The three pinned revisions do NOT expose one identical C++ overload:

* 4.1: camera-matrix overload has no maxIters argument; underlying registrator
  default is 1000.
* 4.10: both legacy no-maxIters and newer maxIters overloads exist. The binding
  uses the legacy overload so both 4.x versions follow the same source route.
* 5.0: legacy overload was removed. Header 3d.hpp lines 1427..1432 declares
  `int maxIters=1000` before OutputArray mask; five-point.cpp 439..489 implements
  that signature. Supplying a mask requires explicitly passing 1000. The focal/
  principal-point overload likewise requires maxIters and is not a workaround.

An actual 5.0 compile probe exposed the original task's incompatible overload
prohibition; the user explicitly approved its correction. A compile-time
`CV_VERSION_MAJOR >= 5` branch calls the required overload with literal **1000**;
4.x calls the legacy overload. No runtime dispatch, public iteration option or
caller-configurable value. Both routes implement ONE fixed-1000 public semantic
contract, not estimator selection. Actual-shim production/fault compilation and
raw profile output on each pinned target prove which route compiled and ran.

In five-point.cpp 4.1 lines 405..449, 4.10 442..499, and 5.0 439..497, observations
are converted to `CV_64F`, reshaped, principal point subtracted and x/y divided
by fx/fy. Threshold is divided by `(fx+fy)/2`. Identity K makes both observations
and threshold retain their normalized units. Neither threshold nor confidence
is substituted at a fundamental-like epsilon boundary.

4.1 invokes `createRANSACPointSetRegistrator(cb,5,threshold,prob)`; precomp.hpp
factory and registrator constructor default maxIters=1000. 4.10 line 498 forwards
its no-max overload explicitly with 1000. The 5.0 public maxIters default is
1000, but supplying an output mask requires explicitly passing that argument.
Adaptive `RANSACUpdateNumIters` can shorten the ceiling.

## Five-point model, exact-five bypass, six-point subsets

`EMEstimatorCallback::runKernel` constructs Q(n,9), the nullspace/SVD, A(10,20),
polynomial coefficient matrices and roots using `CV_64F`/double (4.1 40..145;
4.10/5.0 corresponding kernel). The algebra yields candidate Essential matrices,
possibly stacked, not a Float32 coordinate estimator. This distinction from
Task 005 matters. The binding accepts only the single final 3x3 model selected
by robust consensus; no new rank/singular-value rejection tolerance.

`ptsetreg.cpp` RANSAC run (4.1 153..246, 4.10 160..253, 5.0 159..252):

* count < modelPoints returns false;
* count == modelPoints directly calls runKernel on ALL five, copies its models,
  sets best mask to ones and returns. **No random subsets/consensus**;
* count > modelPoints enters getSubset sampling of five unique random pairs;
* each kernel candidate is tested, strongest support selected, iterations
  adaptively reduced; best single model is copied, otherwise model released.

Ada and raw C minimum six is a binding policy, not an assertion OpenCV cannot
estimate E from five. Armed stage 29 remains pending after five rejects, is
consumed by six, and a subsequent valid call succeeds.

## Error-array boundary and independent public classification

Callback computeError (4.1 373..398; 4.10 382..407; 5.0 380..405) reads Point2d,
calculates Ex1, E^T x2 and residual with Vec3d/Matx33d, but creates `CV_32F` and
stores float `r*r/(Ex1.x^2+Ex1.y^2+Etx2.x^2+Etx2.y^2)`.
Registrator findInliers (4.1 83..99; 4.10 85..101; 5.0 84..100) compares that
Float32 array to `(float)(double_threshold*double_threshold)` using <=.

`essential_profile.hpp` validates precisely this squared threshold conversion
and the registrator's finite strict `(0,1)` confidence interval (4.1 line 167;
4.10 174; 5.0 173). It computes the square-root Sampson metric independently
with checked finite intermediates and nested hypot. Final public classification
uses original normalized Float64 observations and FINAL E, defined error <=
threshold, and requires five final inliers. Native mask is never published.
Unlike Task 005's maximum of two line distances this is the Sampson approximation.

## Pose decomposition, hypotheses, frames and threshold

`decomposeEssentialMat` (4.1 643..666; 4.10 754..777; 5.0 735..758) uses SVD,
corrects U/Vt determinant signs, constructs `R1=U W Vt`, `R2=U W^T Vt`, and
`t=U.col(2)`. That singular-vector column is nominally unit. The binding still
checks all R/t components finite, rejects zero t, robustly normalizes and
publishes only value records. Orthogonality/det/unit tolerances are tests only.

The explicit distance-threshold recoverPose overload is present throughout
(4.1 461..628; 4.10 571..739; 5.0 552..720). It uses P0=[I|0] and
P1..P4=[R1|t], [R2|t], [R1|-t], [R2|-t], triangulates each, checks positive
homogeneous/dehomogenized depth in both cameras and checks depth below the
distance threshold in both cameras. This proves the relative transform:
`X_second = R X_first + lambda*t_hat`, unknown positive scale. Setting X_second
to zero proves second-camera-center direction in first frame is `-R^T*t_hat`,
not OpenCV's translation term. Headers explicitly describe change of basis
first->second and unit/up-to-scale translation.

Every hypothesis mask is intersected with the supplied input mask (4.1 570..579,
4.10 680..690, 5.0 661..671), then support counted. Ordered `>=` comparisons
select maximum support, tie preference P1 then P2 then P3 then P4. 4.10/5.0
reshape input masks more flexibly; our Nx1 CV_8U mask works on all.

Default overloads hardcode 50 (4.1 line 633; 4.10 744; 5.0 725). Binding uses
the common explicit overload with **finite DBL_MAX**, avoiding arbitrary
50-baseline-unit far-point filtering. Depth comparisons are simple `<`
operations; the threshold is not squared/multiplied, so this finite maximum
does not introduce arithmetic overflow. Nonfinite/infinite triangulations can
still fail positivity/depth comparisons. The >50-baseline fixture qualifies
the choice on the pinned runtime matrix. No triangulated output is requested.

The input mask contains exactly stable final-E inliers. Pose support >=5 is a
conservative binding policy; lower support retains E but makes pose unavailable.
Repeated/collinear/zero-parallax probes locally returned nonempty E; portable
no-E/no-pose semantics therefore also use distinct TEST-ONLY suppression after
real estimation/recovery, never production controls. Source review is not
runtime qualification; see `essential-validation.md` for actual results.