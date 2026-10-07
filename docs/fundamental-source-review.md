# Immutable fundamental source review

| Version | Official immutable revision | Native module |
|---|---|---|
| 4.1.0 | `371bba8f54560b374fbcd47e7e02f015ac4969ad` | calib3d |
| 4.10.0 | `71d3237a093b60a27601c20e9ee6c3e52154e8b1` | calib3d |
| 5.0.0 | `40738fb16ceddb5fb3fea747585f7ce6abb0605b` | geometry |

The existing immutable header, fundam.cpp and ptsetreg.cpp bytes were downloaded
again and their SHA-256 and Git blob SHA-1 hashes reverified. precomp.hpp bytes
and hashes are added to `source-provenance.json` for the internal factory default.
All URLs use full commits. No upstream source is vendored into production.

OpenCV 4 paths are `modules/calib3d/include/opencv2/calib3d.hpp` and
`modules/calib3d/src/{fundam.cpp,ptsetreg.cpp,precomp.hpp}`. OpenCV 5 uses
`modules/geometry/include/opencv2/geometry/3d.hpp` and the geometry/src equivalents.

## Declarations, precision and actual estimator branch

Headers declare the six-argument method/threshold/confidence/mask overload on all
targets (4.1 lines 1794..1798; 4.10 2478..2482; 5.0 1379..1383). Later versions
also expose maxIters/USAC overloads; neither is bound here. The headers' enum
comments correctly say FM_RANSAC needs at least 15; their broader function
documentation says N>=8 (4.1 line 1752, 4.10 2430, 5.0 1331). **The actual legacy
implementation, not the broad N>=8 text, defines this binding profile.**

The native implementation converts reshaped input to CV_32F at fundam.cpp lines
787 / 865 / 859, respectively. Public Float64 does not remove that precision
boundary. Native model coefficients and intermediate solver calculations use
double, but point observations have already been rounded to Point2f.

Branches at 798 / 876 / 870 directly solve when npoints==7 (or FM_8POINT).
Otherwise the native robust branch at 816 / 894 / 888 is exactly:

```cpp
if ((method & ~3) == FM_RANSAC && npoints >= 15)
    createRANSACPointSetRegistrator(cb, 7, ...)->run(...);
else
    createLMeDSPointSetRegistrator(cb, 7, ...)->run(...);
```

Thus 7 -> direct seven-point; 8..14 plus FM_RANSAC -> LMeDS; 15+ -> actual
RANSAC. The binding rejects every count below 15 in both Ada and C.
The minimal robust subset size is seven, not eight.

## Iterations and option substitution

OpenCV 4.1 calls the factory without maxIters. precomp.hpp lines 95..97 declares
maxIters=1000; ptsetreg.cpp lines 79..81 agrees. Later versions' no-max overloads
forward explicitly with 1000 (fundam.cpp 4.10 line 910; 5.0 line 904).
There is no source contradiction: the common native ceiling is fixed at 1000,
subject to adaptive shortening by RANSACUpdateNumIters. No public iteration knob.

Threshold <=0 is silently replaced with 3. Confidence checks at fundam.cpp
813 / 891 / 885 are strictly `< DBL_EPSILON` and `> 1-DBL_EPSILON`; replacement
is 0.99. Inclusive endpoints are therefore valid. The binding rejects substitution
values. ptsetreg.cpp findInliers lines 91 / 92 / 92 squares double threshold then
casts to float and includes callback squared errors <= that float. This motivates
the shared squared-binary32 threshold profile, not just a positive float cast.

## Kernel, metric, rank and mask

FMEstimatorCallback (4.1 703..759; 4.10 777..833; 5.0 773..829):

* checkSubset rejects collinearity in either input set.
* runKernel invokes run7Point on seven pairs, otherwise run8Point. Seven-point
  candidates occupy up to three vertically stacked 3x3 CV_64F models.
* computeError constructs F*p1 and F^T*p2, and publishes Float32
  `max(d1*d1/(a1*a1+b1*b1), d2*d2/(a2*a2+b2*b2))` (4.1 742..756;
  4.10 816..830; 5.0 812..826). The binding's maximum point-to-line pixel distance
  is its mathematical square root, using original Float64 observations and scaled
  hypot. It is not Sampson distance.

run7Point obtains a two-dimensional nullspace by full SVD and solves the cubic
det(lambda*F1+(1-lambda)*F2)=0, producing up to three rank-two candidates. The
4.1 implementation (477..583) uses unnormalized coordinates; 4.10 (503..657) and
5.0 (499..653) additionally center/scale and denormalize. This version difference
is another reason to compare geometry rather than coefficients.
run8Point (604..700 / 678..774 / 674..770) centers/scales coordinates, solves the
normal equations, and enforces rank two by SVD with w[2]=0. Bottom-right scaling
is conditional, not an invariant (and seven-point permits zero bottom-right).

PointSetRegistrator RANSAC (ptsetreg.cpp 4.1 153..246; later corresponding run):
uses deterministic local RNG, selects seven noncollinear pairs, evaluates each
candidate separately, accepts a best candidate with at least seven native
inliers, copies one 3x3 bestModel and its bestMask, and adaptively shortens its
ceiling. Failure to find a valid initial subset returns false; no accepted model
releases the output. findFundamentalMat returns empty Mat when result<=0 at
822 / 900 / 894. The raw wrapper treats empty F as successful no-model, never as
an exception. There is no homography-like post-RANSAC LM/refit/mask-update stage
in this legacy fundamental branch. Nonetheless the binding independently
classifies final F using Float64 observations, avoiding Float32 error-mask policy.

Source behavior is not cross-version runtime qualification. Real collinear
no-model and the geometric fixtures are separately tested on the pinned matrix;
see `fundamental-validation.md`.
