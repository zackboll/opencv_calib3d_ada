# Immutable visibility source review

Reviewed upstream revisions and SHA-256 hashes (full downloaded files):

| Version | Revision | homography_decomp.cpp | Public header |
|---|---|---|---|
| 4.1.0 | `371bba8f54560b374fbcd47e7e02f015ac4969ad` | `5f2faba2adc6baaa9d617d48b88b2b23091143e487bde36cbea4e0cd9909601a` | `b525513f3cbdd148aa59feca0da775fe93dd8d2de31f648c473d255d688d415e` |
| 4.10.0 | `71d3237a093b60a27601c20e9ee6c3e52154e8b1` | `5f2faba2adc6baaa9d617d48b88b2b23091143e487bde36cbea4e0cd9909601a` | `f937d4fbd7d9bd0226092e9bc2f4a56c439a27bcf059ee35de299ed2f6ba96f0` |
| 5.0.0 | `40738fb16ceddb5fb3fea747585f7ce6abb0605b` | `dcdaacb7ad796751e284ba80e4782d550c9aed17f0d67af91997b5fa1b0d6e52` | `3e23154729137cce7c1574c7d70d00e1fff4a8d3e1548b4b128d8337e5861c34` |

4.x paths: `modules/calib3d/src/homography_decomp.cpp` and
`modules/calib3d/include/opencv2/calib3d.hpp`. 5.0 paths:
`modules/geometry/src/homography_decomp.cpp` and
`modules/geometry/include/opencv2/geometry/3d.hpp`.
Immutable URLs and existing blob hashes are retained in `source-provenance.json`.

The public declarations accept rotations, normals, before/after points, returned
indices, and an optional mask defaulted to `noArray()`. Translation is absent.
Implementation lines 501–562 (5.0; 502–563 in 4.x) establish:

1. Both observations must be CV_32FC2.
2. Each normal and rotation is converted to CV_64F; `rotnorm=R*normal`.
3. Translation is not used in visibility arithmetic.
4. Dots are `n.x*x1+n.y*y1+n.z` and `(R*n).x*x2+(R*n).y*y2+(R*n).z`.
5. Either dot <=0 clears the candidate flag, over every selected reference.
6. The final ascending loop publishes zero-based original candidate positions.
7. The survivor vector may be empty, copied to the output without an exception.
8. 4.x's mask assertion permits only CV_8U.
9. 5.0 additionally names CV_8S and CV_Bool in its initial mask assertion, but
   still requires `pointsMask.checkVector(1,CV_8U)==npoints` and reads uchar data.
   The header continues to describe an 8u mask. Signed/Boolean masks are not
   inferred to be portable.

4.1 and 4.10 implementation files are byte-identical. The complete 5.0 diff
changes namespace formatting and that initial mask assertion only. Numerical
visibility arithmetic, strict zero behavior and survivor indexing are unchanged.
The binding avoids the discrepancy by compact selection and `noArray()`; native
comparison tests use only CV_8UC1.