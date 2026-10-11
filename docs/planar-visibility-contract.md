# Calibrated homography plane visibility

`Assess_Planar_Visibility` preserves every original hypothesis and returns
`1 .. Hypotheses'Length` in original iteration order. It neither changes the
decomposition nor selects a unique motion. For normalized homogeneous observations
`p1=(x1,y1,1)` and `p2=(x2,y2,1)`, OpenCV tests:

```
n dot p1 > 0
(R*n) dot p2 > 0
```

R maps first-camera coordinates to second-camera coordinates. The second normal
is **R*n, not R^T*n**. Both strict conditions must hold for every selected
correspondence; exact zero rejects. There is no epsilon or sign clamping.
These source-defined plane-normal sign tests are not full Euclidean cheirality,
triangulation, positive-depth proof for arbitrary points, observability, or unique
physical motion selection. More than one candidate may remain. No metric
translation, scale, navigation position, or camera-pose fusion is recovered.

## Dispositions and indices

* `Passes_Visibility`: all tested native signs are positive.
* `Rejected_By_Visibility`: at least one tested sign is nonpositive.
* `Not_Applicable_Pure_Rotation`: the sole native pure/near-rotation hypothesis;
  its exactly zero translation and normal must not be treated as physical rejection.
* `No_Selected_References`: a general candidate with no selected correspondences.

Omitting selection selects all correspondences; explicitly empty selection selects
none. Empty observations likewise select none. Pure rotation remains not applicable
even with no references. Those cases do not invoke the native filter. Zero native
survivors with real references is a successful result, not an exception.
`Visibility_Passing_Hypothesis_Indices` returns only one-based **hypothesis
positions**, not correspondence indices and not absolute Ada array indices.

`Selected_Indices` identifies one-based **correspondence positions**, independent
of observation array lower bounds. Positions must be unique, strictly ascending,
and within 1..N. One selected point is sufficient; no RANSAC minimum applies.
First/second counts must match and fit INT32. All supplied observations must be
finite, including unselected values and values supplied with zero candidates.

## Candidate validation

Zero through four candidates are allowed. Each manually constructed candidate is
validated. General candidates require finite proper R (orthogonality and determinant
errors <=1e-9), finite nonzero t/d, and finite unit n (norm error <=1e-9).
Pure candidates require exactly zero t/d and n, exactly one candidate, and Task
010's native near-rotation tolerance 0.001 for R. The flag is a native classification,
not a proof of exact SO(3). Neither rotations nor translation magnitudes nor normals
are modified. No originating homography is received or reconstructed by this API.
Malformed records and observations raise `OpenCV_Error`, not rejected dispositions.

## Float32 boundary and selection

Public observations are Float64. Selected observations are compacted into Core-owned
Float64 C2 Mats. Callback-scoped Module_Interop borrowing permits the private shim
to create continuous Nx1 CV_32FC2 snapshots. Every converted component must be
finite; overflow and nonzero-to-zero underflow are errors, never clamped.
Finite nonzero Float32 subnormals are permitted. Classifications reflect actual
native Float32 observations; Float64 sign decisions can differ near zero.

The native call uses `noArray()` for its mask, selecting all compact references.
No public mask interface is exposed. A separate native qualification fixture
compares compact observations against full observations with a CV_8UC1 mask.
Only CV_8U is portable across the reviewed versions; see the source review.

The caller-owned private result is cleared at entry, staged locally, and published
only after all input, conversion, native-call and returned-index checks. Native
indices must be an ascending unique CV_32SC1 vector within candidate bounds.
All native exceptions remain behind the existing C ABI guard.