# Calibrated planar motion contract

`Decompose_Calibrated_Homography (Matrix, Intrinsics)` returns owned,
one-based `Planar_Motion_Hypothesis_Array` values in native order. No hypothesis
is selected automatically. Task 011 adds separate visibility assessment; see
[visibility contract](planar-visibility-contract.md).

For an undistorted pinhole-image homography with the same intrinsics in both
views, `K = [fx,0,cx; 0,fy,cy; 0,0,1]`, the relationship is

```
K^-1 H K ~ R + (t/d) n^T
X_second = R X_first + t
n^T X_first = d
```

`R` is first-camera to second-camera rotation; `n` is in the first-camera
frame. `Translation_Over_Plane_Distance` is the native translation term divided
by unknown plane distance. Its magnitude is preserved: it is not a unit
translation direction, camera position, or metric baseline. Simultaneously
negating `t/d` and `n` preserves their outer product. The collection represents
alternative mathematical motions, not unique physical poses. Use identity
intrinsics for homographies already expressed in normalized coordinates.

Focal lengths must be finite and positive, principal points finite. There is
no two-intrinsics overload and no implicit distortion correction.

## Scale and representability

Before native entry, finite coefficients are divided by their maximum absolute
value. Nonzero-to-zero underflow is rejected. The determinant is evaluated in
long double; singular matrices are rejected and negative orientation negates
the whole matrix. This removes negative common scale without dividing by h22.
Calibrated intermediates must remain finite, with positive middle and smallest
singular values. Ill-conditioned/unrepresentable inputs can raise `OpenCV_Error`;
this is not a claim that every finite nonsingular matrix is numerically solvable.
No invalid inputs are silently clamped.

Native scale removal divides by the middle singular value. Every candidate
must have the exact Float64 native schema and finite fields. General candidates
must have rotation orthogonality/determinant errors at most 1e-9, nonzero
translation, and unit normal to 1e-9. No translation renormalization occurs.

## Native near-rotation behavior

OpenCV tests `norm(Hnorm^T Hnorm-I, INF) < 0.001`. This returns one candidate
with zero translation and normal, marked `Pure_Rotation=True`. Exact pure
rotation fixtures return proper rotations to floating-point precision.
Near-rotations return Hnorm directly, without projection onto SO(3). The binding
preserves that result, requiring orthogonality/determinant error at most 0.001;
`Pure_Rotation` is a native classification, not a proof of exact physical rotation.
The discarded near-rotation rank-one component need not reconstruct the input
exactly. General candidates reconstruct the normalized projective geometry.

## Errors and ownership

One legitimate zero-solution result would return `1 .. 0`. No reproducible real
native zero-solution case is claimed: tests explicitly override output after a
real native call. Native errors, bad counts, schemas, nonfinite values, and
invalid rotations raise `OpenCV_Error`, not successful empty arrays.

The private ABI has caller-owned fixed storage: an int32 count and four
fifteen-double candidates. The output is cleared before validation; every
candidate is validated in local storage before atomic publication. All C++
exception categories are caught. No native handles or STL types are public.