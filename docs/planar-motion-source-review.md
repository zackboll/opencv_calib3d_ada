# Immutable homography decomposition source review

Reviewed the public headers and `homography_decomp.cpp` at these immutable
OpenCV revisions; full URLs, SHA-256 and Git blob hashes are recorded in
`source-provenance.json`.

| Version | Commit | Module | Implementation SHA-256 |
|---|---|---|---|
| 4.1.0 | `371bba8f54560b374fbcd47e7e02f015ac4969ad` | calib3d | `5f2faba2adc6baaa9d617d48b88b2b23091143e487bde36cbea4e0cd9909601a` |
| 4.10.0 | `71d3237a093b60a27601c20e9ee6c3e52154e8b1` | calib3d | `5f2faba2adc6baaa9d617d48b88b2b23091143e487bde36cbea4e0cd9909601a` |
| 5.0.0 | `40738fb16ceddb5fb3fea747585f7ce6abb0605b` | geometry | `dcdaacb7ad796751e284ba80e4782d550c9aed17f0d67af91997b5fa1b0d6e52` |

4.x uses `modules/calib3d/include/opencv2/calib3d.hpp` and
`modules/calib3d/src/homography_decomp.cpp`; 5.0 uses
`modules/geometry/include/opencv2/geometry/3d.hpp` and
`modules/geometry/src/homography_decomp.cpp`.

`HomographyDecomp::normalize` computes `K.inv()*H*K`.
`removeScale` obtains the singular values and multiplies Hnorm by `1/W[1]`.
This does not remove negative projective sign. The binding therefore checks
and canonicalizes orientation before entry.

The exported function constructs `HomographyDecompInria`, not the alternative
Zhang implementation. Its `decompose` computes `S=Hnorm^T Hnorm-I` and compares
the infinity norm with `epsilon=0.001`. The near-rotation branch copies Hnorm
and explicitly sets translation and normal to zero, returning one solution.

The general branch normalizes na/nb; `findRmatFrom_tstar_n` computes
`R=Hnorm*(I-(2/v)*tstar*n^T)`, flips R if its determinant is negative, and
sets `ta=Ra*ta_star`, `tb=Rb*tb_star`. It resizes to four and writes
`(Ra,ta,na)`, `(Ra,-ta,-na)`, `(Rb,tb,nb)`, `(Rb,-tb,-nb)`.
Independent fixtures verify `Hnorm ~ R+(t/d)n^T`, with the normal in the
first camera and translation term in the second-camera transform.

`CameraMotion` uses `Matx33d` and `Vec3d`. Exported outputs have CV_64F depth,
3x3 single-channel rotations and 3x1 single-channel translation/normal matrices.
The input wrapper reshapes to three rows and asserts 3x3 dimensions. The
algorithm has no comprehensive finite/singularity validation: zero middle
singular value, square roots, and normal division can yield invalid arithmetic
rather than clean no-solution output. Such inputs/output are guarded by the
binding; all exceptions stay inside the C ABI. No arbitrary degeneracy is
documented as naturally returning zero solutions.