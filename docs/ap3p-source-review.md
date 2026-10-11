# AP3P immutable source review

Reviewed at the pinned peeled revisions. SHA-256 and Git blob hashes were
computed from those checkouts.

| Version | Revision | Backend |
|---|---|---|
| 4.1.0 | `371bba8f54560b374fbcd47e7e02f015ac4969ad` | calib3d |
| 4.10.0 | `71d3237a093b60a27601c20e9ee6c3e52154e8b1` | calib3d |
| 5.0.0 | `40738fb16ceddb5fb3fea747585f7ce6abb0605b` | geometry |

`solveP3P` on 4.1 asserts exactly three points. On 4.10 and 5.0 it asserts
three or four. All three call `undistortPoints` and then `ap3p::solve`.
4.10 and 5.0 then sort surviving poses by reprojection error. 4.1 does not.
`SOLVEPNP_AP3P` is 5 on OpenCV 4 and 3 on OpenCV 5; the binding uses the
enumerator, not a copied integer.

`ap3p.cpp` is identical between 4.10 and 5.0. 4.1 uses the same pose algebra
with a local quartic solver and no fourth-point path. `computePoses` stores
at most four solutions in fixed arrays.

Unsafe native divisions and products, all guarded by exact-zero or
nonfinite preflight rather than by copying the solver:

| Native operation | Rejected input |
|---|---|
| `vect_divide(u0, nu0)` | zero world baseline, including repeated points |
| `vect_divide(nl, delta)` | collinear world points (`u1 × k1 = 0`) |
| `vect_divide(k3, nk3)` and bearing normalization | identical or parallel first two bearings |
| `delta / k3b3` | third bearing in the plane of the first two |
| quartic products and `solve_deg4` division by `R` | nonfinite or overflowing intermediates |
| `1/norm` after undistorted normalization | nonfinite pixels or nonpositive focal length |

`polishQuarticRoots` divides by a derivative. That derivative is data
dependent and is not given a universal safe epsilon. The binding therefore
does not claim every finite admissible triple is free of native division;
it rejects the structural zeros above and validates every returned pose.
A nonfinite or non-rotation native candidate becomes `OpenCV_Error`, not
a published hypothesis.

The binding does not reimplement AP3P.
