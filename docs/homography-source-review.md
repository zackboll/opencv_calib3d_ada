# Immutable native homography source review

Official immutable OpenCV revisions:

| Version | Revision | Header / implementation module |
|---|---|---|
| 4.1.0 | `371bba8f54560b374fbcd47e7e02f015ac4969ad` | calib3d |
| 4.10.0 | `71d3237a093b60a27601c20e9ee6c3e52154e8b1` | calib3d |
| 5.0.0 | `40738fb16ceddb5fb3fea747585f7ce6abb0605b` | geometry |

The immutable public headers were downloaded and their existing SHA-256 and Git
blob SHA-1 values independently reverified. `fundam.cpp` and `ptsetreg.cpp` were
downloaded from the same revisions; their byte-derived SHA-256/Git blob values and
immutable URLs are added to `source-provenance.json`. No upstream files are
vendored into production.

Paths for OpenCV 4:

```text
modules/calib3d/include/opencv2/calib3d.hpp
modules/calib3d/src/fundam.cpp
modules/calib3d/src/ptsetreg.cpp
```

Paths for OpenCV 5:

```text
modules/geometry/include/opencv2/geometry/3d.hpp
modules/geometry/src/fundam.cpp
modules/geometry/src/ptsetreg.cpp
```

## Declaration and estimator flow

All three public headers declare the legacy method/threshold/mask/maxIters/
confidence overload. Header documentation specifies forward L2 reprojection error,
four-pair robust subsets, subsequent refinement, projective scale ambiguity and
empty H on failure. The binding uses only `cv::RANSAC` in this overload, not the
USAC overload/parameter object or RHO/LMEDS branches.

`findHomography` entry points in immutable `fundam.cpp` begin at lines 350 (4.1),
357 (4.10), and 296 (5.0). They validate point vectors, reshape, then convert both
point sets to CV_32F at lines 377 / 391 / 327. The older 4.1 entry lacks the later
explicit `<4` diagnostic; the shared registrator rejects insufficient support.
The binding independently validates Nx1 CV_64FC2 and counts, avoiding reliance on
these differences. Noncontinuous input is snapshotted before reshape.

The branches `method == 0 || npoints == 4` at lines 387 / 401 / 337 create an
all-ones mask and call `runKernel` directly. Therefore a method argument of RANSAC
alone does **not** guarantee robust consensus for four pairs. Binding minimum is
deliberately five in both public Ada and raw C.

RANSAC calls `createRANSACPointSetRegistrator(cb,4,threshold,confidence,maxIters)`
at lines 393 / 407 / 343. Registrator `findInliers` calls `computeError`, creates
CV_8U mask, converts squared threshold to float, and includes squared errors <=
that value. Homography `computeError` uses Float32 point/model arithmetic and
Float32 squared forward L2 distance (4.1 lines 191..208; corresponding callbacks
in later versions). This is **not** the binding's final Float64 hypot classifier.
Subset checks reject collinear points and inconsistent minimal-set orientation.

## Final refinement and the mask-version finding

After native consensus, each version compresses points using the native mask,
refits H on accepted points, then refines it:

- **4.1** lines 401..415: refit, eight-parameter LMSolver, ten iterations;
  lines 418..421 copy the **unchanged earlier mask** to the caller.
- **4.10** lines 415..430: refit, nine-parameter LMSolver, ten iterations,
  normalize through `scaleFor`; lines 433..436 copy the **unchanged earlier mask**.
- **5.0** lines 351..429: save original *Float32* points, compress/refit, run
  nine-parameter LevMarq (ten iterations with geodesic setting), normalize; then
  lines **420..428 recompute errors and update every mask entry against final H**.

This is a real semantic difference, not a claim that a particular artificial
fixture must make the versions disagree. Independent final-H classification is
the binding portability boundary. Both includes and excludes are verified against
the final returned model, using the original caller Float64 data and robust hypot.

## Matrix scale and failure

The DLT kernel produces a 3x3 CV_64F model after independent coordinate
normalization. 4.1 scales by `1/H(2,2)` (line 178). 4.10/5.0 use `scaleFor`, which
leaves very small bottom-right coefficients unnormalized, including after LM.
Consequently `h22=1` must not become a public invariant. The binding accepts any
nonzero common scale, validates all nine coefficients, and checks mappings rather
than coefficient equality.

On estimation failure the implementations release H and create a zero mask
(4.1 lines 423..428; 4.10 438..443; 5.0 437..442). This is a legitimate no-model
result. Invalid nonempty matrices are instead binding errors. The exact collinear
fixture is native evidence locally; its cross-version behavior is qualified by
the pinned matrix, not merely inferred from source. No forced no-model control
is introduced unless observed native version behavior requires one.

See `homography-contract.md` for the binding policy and `homography-validation.md`
for executed results. Source review alone is not a build/test qualification.