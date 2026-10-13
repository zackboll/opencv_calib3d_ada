# Guarded native epipolar correspondence correction — contract

`Correct_Epipolar_Matches (Matrix, First_Points, Second_Points, Options)` binds
native OpenCV `cv::correctMatches` (Hartley–Sturm optimal correction) through one
guarded private C operation, `opencv_calib3d_correct_matches`.

**Native `cv::correctMatches` performs the algorithm.** Neither Ada nor the C++
shim recreates Hartley–Sturm, solves the degree-six polynomial, or offers a
fallback/competing correction. Ada and the shim each validate inputs; the shim
normalizes F, screens conditioning and input lines, runs the native call once,
and verifies the output before publishing.

## What success means — and what it does not

A successful return means exactly:

1. Inputs and options passed validation and the caller's numerical-admissibility policy.
2. The actual OpenCV algorithm executed.
3. Every returned coordinate is finite and structurally valid.
4. Each corrected pair meets the caller's epipolar-consistency requirement.
5. Each corrected pair's displacement passes the one-sided bound, which is a **necessary condition**
   for optimality.

A successful result is **not a global-optimality certificate**. It does not guarantee:

* the global mathematical optimum (a suboptimal native result can stay below the bound);
* reliable output for every mathematically valid F;
* accurate triangulated 3-D positions, correct correspondence identity, camera pose,
  metric scale, baseline, geographic position, or navigation-quality estimates.

Native output may be **rejected despite a successful OpenCV return**: OpenCV
reports no numerical failure and can return NaN, the unchanged input, or finite
but wrong points (see [source review](epipolar-correction-source-review.md)).
Near-degenerate geometry (near rank one, near epipoles, extreme coordinates)
remains risky. Results depend entirely on the supplied F and the pixel
coordinate domain; the operation never estimates or replaces F, runs RANSAC,
undistorts, or selects correspondences. Calibration/undistortion are the
caller's responsibility.

## Options (all caller-selected; no defaults)

Let `s1 >= s2 >= s3 >= 0` be the measured singular values (native Float64 SVD)
of F divided by its maximum absolute coefficient. No separate rank-two projection
is applied.

| Field | Meaning | Valid range | Accept if |
|---|---|---|---|
| `Minimum_Second_Singular_Ratio` | numerical rank / conditioning | `0 < v <= 1` | `s2/s1 >= v` |
| `Maximum_Third_Singular_Ratio` | distance from rank two | `0 <= v < 1`, `v < minimum second` | `s3/s1 <= v` |
| `Minimum_Epipolar_Line_Ratio` | input line quality (below) | `0 <= v <= 1` | `quality >= v` |
| `Maximum_Corrected_Error_Pixels` | corrected symmetric error | `v >= 0` | `error <= v` |
| `Absolute_Displacement_Excess_Pixels` | bound tolerance | `v >= 0` | see bound |
| `Relative_Displacement_Excess` | bound tolerance | `0 <= v <= 1` | see bound |

All values must be finite. Both Ada and the shim enforce these ranges
independently. Comparisons are inclusive as written above; tests exercise exact
boundaries and one-ulp neighbours. These ranges describe the meaning of each
option. No combination of values guarantees optimality, and no value is a universal safe default.
Stricter policies reject more inputs. Example values used in tests and the example
program are **application choices justified by measured fixtures only**.

## Fundamental-matrix normalization

The shim reads all nine coefficients, rejects nonfinite values and the zero matrix,
divides by the maximum absolute coefficient, and rejects the call if any nonzero
coefficient would become zero or subnormal (the supplied matrix would otherwise
silently change). Sign and `F(2,2)` are not altered. Ada performs the same
normalization for its own diagnostics so extreme-scale F cannot overflow them.
Normalization preserves homogeneous geometry; it is not a substitute for
`correctMatches`.

## Input epipolar-line quality

For `p1=(x1,y1,1)`, `p2=(x2,y2,1)`, `l2=F p1`, `l1=F^T p2` (normalized F):

```text
quality = min( hypot(l2.x,l2.y) / (||F||_F * ||p1||),
               hypot(l1.x,l1.y) / (||F||_F * ||p2||) )
```

The measure is dimensionless, lies in `[0,1]`, and is invariant to nonzero common
scaling of F and of each homogeneous point. Norms use scaled `hypot`. Nonfinite or
nonrepresentable line arithmetic and zero line normals (exactly at an epipole)
are rejected before native entry. There is no additional hidden near-epipole
epsilon. This is a numerical quality measure, not a geometric correctness proof.
It also depends on coordinate magnitude: very large pixel coordinates lower the
quality for the same geometry.

## Output verification

After the single native call, the shim requires each native output to be
`1xN CV_64FC2` with matching count and all coordinates finite. For each pair,
using the normalized F, it then requires:

* both corrected epipolar lines are defined, and
  `Corrected_Error = max(dist(q1, F^T q2), dist(q2, F q1)) <= Maximum_Corrected_Error_Pixels`;
* the one-sided displacement bound, which is a necessary condition only:

```text
d1 = dist(p1, F^T p2)     d2 = dist(p2, F p1)     Bound = min(d1, d2)
D  = || (q1 - p1, q2 - p2) ||_2       (scaled 4-D norm, no unscaled squaring)
D <= Bound * (1 + Relative_Displacement_Excess) + Absolute_Displacement_Excess_Pixels
```

Moving only one point onto its epipolar line is a feasible correction, so the
true optimum cannot move further than `Bound`. Exceeding it proves the native
result is not optimal. Staying below it proves nothing more. If the allowance
cannot be represented, the call is rejected; it never silently passes.

The initial API is **all-or-error**. If any pair fails any check, the entire call
raises `OpenCV_Error` with no partial arrays, no skipped pairs, and no substituted
or original-point fallback. An unchanged correspondence is acceptable only if it
passes the same checks.

Ada independently re-verifies every published pair with `Maximum_Epipolar_Error`
and its own scaled displacement. Any disagreement is an error.

## Public results

`Corrected_Epipolar_Matches (Count)` owns one-based `First_Points`,
`Second_Points`, and `Diagnostics` arrays. Inputs with any lower bounds are
paired by position. Each diagnostic records the original and corrected symmetric
errors (`Maximum_Epipolar_Error`, not Sampson), the total displacement, and the
one-sided bound. These are measured geometric values, not optimizer outputs or
certificates, and every one is finite. No native handle escapes.

Empty/empty input returns `Count = 0`. F, options, and the SVD policy are still
validated by the shim, so invalid F never becomes an empty success. Mismatched
lengths raise `OpenCV_Error`.

## Private C ABI and Core Mats

* Reuses the nine-double `opencv_calib3d_fundamental`. Adds the private
  six-double `opencv_calib3d_correction_options` (size 48, alignment 8, offsets
  0..40, compiler-qualified in C and Ada).
* Adds exactly one export: `opencv_calib3d_correct_matches`. The private ABI grows
  from 47 to **48** operations.
* Inputs are `Nx1 CV_64FC2` (typed `0x1` when empty). Valid strided Core Regions
  are snapshotted into independent contiguous `1xN CV_64FC2` matrices, which is
  the layout the legacy 4.x solver requires; 5.0 accepts it too. The output is
  `Nx2 CV_64FC2` (column 0 = corrected first, column 1 = corrected second), or
  typed `0x2` when empty.
* Failure atomicity: output handles equal to an input handle are rejected before
  output resolution. A distinct output handle that shares storage with an input is
  safe, because inputs are fully snapshotted and validated before publication and
  publication only rebinds the output header. The complete result is built locally
  and published once. On any failure the caller's output Mat is unchanged. Inputs
  are never mutated, and no C++ exception crosses the ABI.
* Fault checkpoints: 58 (before native), 59 (after native), 60 (before output
  validation), 61 (immediately before publication; also on the empty path). 57
  was the previous maximum. A test-only post-native control (fault builds only)
  alters a real native result to exercise the output gate.
