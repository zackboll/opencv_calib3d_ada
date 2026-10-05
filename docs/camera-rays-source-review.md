# Task 003 immutable source review

The reviewed revisions remain:

| OpenCV | Official peeled commit | Standard undistortion implementation |
|---|---|---|
| 4.1.0 | `371bba8f54560b374fbcd47e7e02f015ac4969ad` | `modules/calib3d/src/undistort.cpp` |
| 4.10.0 | `71d3237a093b60a27601c20e9ee6c3e52154e8b1` | `modules/calib3d/src/undistort.dispatch.cpp` |
| 5.0.0 | `40738fb16ceddb5fb3fea747585f7ce6abb0605b` | `modules/geometry/src/pinhole.cpp` |

Immutable raw URLs, SHA-256, and Git blob SHA-1 are in `source-provenance.json`.
Task 003 re-fetched and verified existing reviewed header/Rodrigues file hashes,
then added the three undistortion implementation hashes. OpenCV 5's image-remap
implementation in imgproc is not the sparse geometry operation this crate binds.

## Standard undistortPoints

* 4.1 public calib3d.hpp 2717/2724 declares the default and explicit criteria
  overloads. `CV_EXPORTS_AS(undistortPointsIter)` is a wrapper-language annotation,
  not a differently named C++ call. undistort.cpp 558-582 is the explicit overload;
  371-535 is the double iterative kernel.
* 4.10 calib3d.hpp 3750/3757 retains both overloads. undistort.dispatch.cpp
  385-556 is the kernel, 578-608 the explicit overload. It uses `checkVector(2)`
  and creates Nx1 output at input depth (Float64 C2 for our schema).
* 5.0 geometry/3d.hpp 2358-2362 merges criteria into the single standard signature
  with a default. pinhole.cpp 106-318 is the kernel, 321-343 the wrapper. The
  seven-argument call is source-compatible and creates Nx1 C2 at input depth.

All three require continuous Float32/Float64 vector input; 4.1 explicitly asserts
rows/channels or cols*channels, later versions use `checkVector(2)`. Our narrow
Nx1 CV_64FC2 preflight is stricter; strided valid Core headers are cloned. The 4.1
output retains input size/type, which is Nx1 C2 for this contract.

The kernels convert K/distortion to double, use `(u-cx)/fx,(v-cy)/fy`, set omitted
R to identity, and do not apply a new projection when P is omitted. Double input
uses double point reads/writes. Five coefficients select radial K1/K2/K3 and
tangential P1/P2; rational/thin-prism/tilt terms remain zero and are not bound.
An empty coefficient Mat skips iteration and is the documented zero model.

With COUNT|EPS, kernels stop at the count cap or when the forward-projected
pixel error is below epsilon. The common explicit policy is 20 and 1e-12.
4.10/5 add a negative inverse-radial fallback not present in 4.1; extreme
distortion does not have a common convergence guarantee. Iteration exhaustion
returns an approximation, not a failure flag. Assertions/schema/type failures
throw cv::Exception; arithmetic can return nonfinite values. The shim separately
validates all output coordinates before publication and catches every exception.

The moderate all-five independent fixture uses 1e-10 normalized/direction
component tolerance, not bit-identical outputs or a universal error bound. Source
review explains the fixed policy; pinned execution provides cross-version evidence.

## Rodrigues

Declarations are calib3d.hpp 318 (4.1), 603 (4.10), geometry/3d.hpp 465 (5.0).
4.1/4.10 calibration.cpp public wrappers at 3479/3635 create output at input
depth and invoke `cvRodrigues2` (251/252). The 3x1 CV_64F vector branch returns
3x3 CV_64FC1. Below DBL_EPSILON it writes identity; otherwise it computes the
axis-angle expression with sine/cosine and normalized axis. 5.0
geometry/src/calibration_base.cpp 121-198 retains this behavior in direct C++.
No Jacobian is requested by the new operation.

Native vector-to-matrix arithmetic is not a finite/range guarantee: enormous
finite rotation vectors can overflow norm and produce NaN. The shim requires
all six pose components finite, validates 3x3 CV_64FC1 and every result component,
and leaves its C output zero on failure. Orthonormality/determinant are qualification
oracles, not added native rejection policy. Ada uses the qualified matrix for
small point/direction arithmetic, never reimplements Rodrigues.