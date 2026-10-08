# Immutable triangulation source review

| Version | Commit | Implementation |
|---|---|---|
| 4.1.0 | `371bba8f54560b374fbcd47e7e02f015ac4969ad` | `modules/calib3d/src/triangulate.cpp` |
| 4.10.0 | `71d3237a093b60a27601c20e9ee6c3e52154e8b1` | `modules/calib3d/src/triangulate.cpp` |
| 5.0.0 | `40738fb16ceddb5fb3fea747585f7ce6abb0605b` | `modules/geometry/src/triangulate.cpp` |

Downloaded source SHA-256 and Git blob hashes are recorded separately in
`source-provenance.json` for these triangulation implementations.

4.x uses CvMat adapters. Nx1 C2 input may be reshaped/transposed, a path avoided
here. Output has four rows and point input type. `icvTriangulatePoints` requires
3x4 projections, 2xN observations, 4xN output, and N>0. Its double Matx system
uses `cv::SVD::compute`; the last right singular vector is stored with cvmSet.

5.0 accepts matching CV_32F or CV_64F point depth, reads planar inputs using row
stride, converts projections to double Matx, invokes `hal::SVD64f` directly,
and stores CV_64F homogeneous results with `at<double>`. No legacy CvMat path.

The older Float32-oriented documentation note is not execution evidence and
cannot replace native Float64 execution qualification for each pinned version.
Task 007 native execution currently qualifies 4.10.0 only. 4.1.0/5.0.0 remain
pending. Empty native behavior differs; this binding avoids empty native entry.
Snapshots and explicit planar copies avoid strided reshape assumptions.

Task 006's recoverPose convention remains `X2=R X1+t`; camera center is `-R^T t`.
The inverse-frame oracle independently reverses this transform.