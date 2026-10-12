# Pose composition native source review

Immutable sources (tag peeled to commit; SHA-256 and git blob SHA-1 in
[source-provenance.json](source-provenance.json) under `task_014_review`).

```text
4.1.0   371bba8f54560b374fbcd47e7e02f015ac4969ad  calib3d
4.10.0  71d3237a093b60a27601c20e9ee6c3e52154e8b1  calib3d
5.0.0   40738fb16ceddb5fb3fea747585f7ce6abb0605b  geometry
```

The 4.1.0 hashes equal the pre-existing provenance entries.

## Findings

1. **Shapes and types.** Public signature (all versions) is
   `composeRT(rvec1, tvec1, rvec2, tvec2, rvec3, tvec3, 8 optional Jacobians)`.
   4.x forwards to legacy `cvComposeRT`, which requires 32F or 64F and a 3x1
   `rvec1` with `rvec2` the same size (4.1 `calibration.cpp` line 149; 4.10 line
   150). 5.0 (`calibration_base.cpp` 373) is a new inline implementation that
   accepts 3x1 or 1x3 and checks all four sizes match. The binding passes 3x1
   CV_64FC1 only, which both accept.
2. **Input rotation.** Rodrigues vectors, converted by `Rodrigues`.
3. **Output rotation.** Rodrigues vector from `Rodrigues(R3)`, output type and size follow
   `rvec1` (3x1 Float64 here).
4. **Rotation order.** `R3 = R2 * R1` in both implementations.
5. **Translation order.** `t3 = R2 * t1 + t2` (`cvMatMulAdd` / `gemm`).
6. **Zero rotation.** Vector-to-matrix returns identity when `theta < DBL_EPSILON`.
   Zero composed with zero is exactly zero.
7. **Near 180 degrees.** Matrix-to-vector uses SVD re-orthogonalization and a
   `s < 1e-5` branch; for `c <= 0` it reconstructs the axis from diagonal
   terms. Branch (sign) choice is not guaranteed; compare matrices.
8. **Large rotation vectors.** `theta = norm(r)` is a plain double norm. Large
   finite magnitudes are periodic through `sin`/`cos`; precision is lost, not
   correctness of the matrix as a rotation. If the squared sum overflows,
   `theta` is infinite and `cos` is NaN.
9. **Large translations.** `R2 * t1 + t2` is plain double arithmetic and can
   overflow to infinity.
10. **Exceptions.** `CV_Assert` / `CV_Error` raise `cv::Exception`. Wrong size or
    type fails there. No exception is thrown for nonfinite values.
11. **Output guarantees.** None for finiteness. The native code may return NaN or
    infinity without an error.
12. **Version differences.** 4.1 and 4.10 `cvComposeRT` bodies are identical apart
    from the enum spelling. 5.0 changes the structure (inline) but uses the same
    equations and the same Rodrigues thresholds. No numerical difference was
    found in source review. This is a source finding; the pinned matrix supplies
    execution evidence.

## Qualification probes (installed OpenCV 4.10.0)

```text
rotation 1e200 or 1.7e308 (squared sum overflows)  -> r3 = (0,0,0), NaN-free but WRONG (rotation lost silently)
1e160 on all three axes and the other argument 1e160 -> t3 = NaN
translations of 1.7e308 on both sides               -> t3 = inf
rotation 1e10                                       -> finite, valid rotation
pi-1e-9 composed with itself                        -> r3 = 0 (physical angle 2e-9 flushed)
0.5 then -0.5+3e-5                                  -> r3x = 3e-5 (above the 1e-5 flush)
```

## Numerical-safety conclusion

Wrong-but-finite output arises when `rx*rx+ry*ry+rz*rz` overflows: `Rodrigues`
sees an infinite angle, the later matrix range check silently writes zero. The
binding therefore rejects exactly that condition before native entry. It is
derived from the source arithmetic and the probes, not an arbitrary angle limit.
The condition tests the same quantity the native norm uses, so it is neither more
nor less strict than the native hazard for those intermediates.

The remaining hazard (translation overflow) is detected on the output by the
finite check. Finite inputs whose rotation norm is large but not overflowing are
accepted; their matrices are valid rotations. The binding makes no claim about
angular precision at such magnitudes.

Unresolved: only the 4.10.0 probes were measured locally. 4.1.0 and 5.0.0
execution is established by the pinned matrix.
