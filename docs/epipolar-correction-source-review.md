# Epipolar correction (`cv::correctMatches`) — source review and safety research

Status: RESEARCH ONLY. No production binding exists. Disposition: **SAFETY_GATE_NOT_ESTABLISHED**
(pending independent review; see "Missing proof obligations").

## Pinned sources (builds from shallow pinned tags, `WITH_ADE=OFF`, Release, GCC 14.2, C++17)

| OpenCV | Commit | Native file | SHA-256 | git blob |
|---|---|---|---|---|
| 4.1.0 | 371bba8f54560b374fbcd47e7e02f015ac4969ad | modules/calib3d/src/triangulate.cpp | 190fa8e99e8b799523e2f2522f51b6b7bdb5e3e56e2ded2c13d8edd54d9c9515 | cded42232aada74a8b7806ca19198ee12461e574 |
| 4.10.0 | 71d3237a093b60a27601c20e9ee6c3e52154e8b1 | modules/calib3d/src/triangulate.cpp | 5d8d8970de046dbe3e981dcda7581bb90ba746b9180ee90927238444489b938d | 671659c84b7a77a335de61af8bae752fb6e88dd9 |
| 5.0.0 | 40738fb16ceddb5fb3fea747585f7ce6abb0605b | modules/geometry/src/triangulate.cpp | 8f1adccf184e2b1dc09867100a9ea38af3d1883ecf77b0609b8c27e990012b8f | 3a0809b30339fcf1ad10ec6acbfd8cec822b04ce |

4.1.0 and 4.10.0 differ only in `CV_Error` constant spellings (same algorithm).

## Native behaviour (source)

* 4.x: `icvCorrectMatches`; requires 1xN, 2-channel points; F 3x3 single channel; converts to double.
  Throws on schema errors only. 5.0: `Matx` rewrite; accepts 1xN or Nx1; same polynomial and divisions.
* Per point: translate to origin, two SVDs for epipoles, `scale = sqrt(V02^2+V12^2)` and
  division by it (epipole at infinity -> scale 0 handled only because the other factor stays finite),
  degree-6 polynomial (coefficients are degree <= 8 in F entries, so F scale enters as s^4..s^8),
  `solvePoly`, then `s_val` initial = `1/f1^2 + c^2/(a^2+f2^2 c^2)` and `t_min = DBL_MAX` default.
* No validation of F finiteness, rank, scale, or of output finiteness. `t_min` stays `DBL_MAX`
  if no root beats the initial `s_val`; recovery divisions then yield NaN or the input.
* Nothing throws for numerical failure; NaN is returned silently.

## Executed probes (ASan/UBSan on the probe only; OpenCV libraries are NOT instrumented)

Probes: `/tmp/ocv/probe/{lib.hpp,q.cpp}`, 90 cases x 3 versions plus Debian 4.10.0 control.
All runs exited 0 with empty sanitizer stderr.

* Raw F scale: 1e-40, 1e-100, 1e40, 1e100 silently return the INPUT unchanged on 4.x;
  1e-300 returns NaN; 1e300 overflows the probe's own SVD and returns NaN. 5.0 at 1e100 returns a
  wrong non-optimal result (disp 5.97 vs optimum 2.83).
* Dividing F by its maximum |coefficient| (`lost=0`, no subnormals in every case run) gives the
  analytic optimum (100,22)/(90,22) for F, -F, 2F, 0.5F, 1e+-40, 1e+-100, 1e+-300 on all three versions.
* Zero, NaN, Inf F and single-entry F (F22, F02, F20, F12, F21 only) return NaN on all versions.
* Rank: full rank with sv3/sv1 >= 1e-8 returns finite points with relative residual up to 5e-6;
  sv3/sv1 <= 1e-12 is indistinguishable from rank 2. Rank 1 returns finite but WRONG points
  (sym error ~300, displacement 54.3 vs one-sided feasible bound 11.1) on all versions.
  Near-rank-1 rank-2 F (sv2/sv1 <= 1e-8) is also wrong; 5.0 degrades earlier (1e-12 -> sym 13).
* Epipole: point exactly at an epipole -> NaN or undefined line; 1e-9 from the epipole the 4.x
  result is non-optimal (disp 0.170 vs feasible bound 6.4e-10) while 5.0 is near-optimal.
* Extreme coordinates, general rank-2 F: 4.1.0/4.10.0 return NaN from coordinates 1e6 up
  (rectified F survives to 1e154); 5.0.0 is correct to 1e30 and NaN at 1e100. Rectified F at 1e150 in
  5.0.0 returns unchanged input with huge residual.
* Repeated and identical correspondences: correct on all versions.
* Task 014 pinned matrix run 38198320642 (head 06d1ac4) completed: 4.1.0, 4.10.0, 5.0.0 all success.

## Output-validation findings

Scale-free metric `max sym_out/(1+max|input coord|)` with F normalized:

* well-conditioned rank-2: <= ~3e-16 on every version;
* rank-2 with sv2/sv1 = 1e-3: ~5e-13; 1e-8: ~5e-8 (still "plausible"); 1e-12: 0.002-0.13; rank 1: ~1;
* near-epipole 1e-9 (4.x): 9e-6; undefined epipolar line: inf.

A necessary-optimality check that needs no solver: the native displacement must not exceed the
cheaper one-sided projection (move only p1 or only p2 to its epipolar line). It flagged every
rank-1, near-rank-1, near-epipole(4.x) and 5.0 raw-1e100 failure above, with no false positive
among the accepted fixtures. It cannot prove optimality.

## Missing proof obligations

1. No source-derived residual tolerance: the legitimate residual grows with F conditioning
   (1e-16 -> 5e-13 -> 5e-8), so any fixed epsilon either rejects valid F or accepts wrong output.
2. Rank-2 admissibility has no principled cutoff; sv2/sv1 between 1e-3 and 1e-12 is unresolved.
3. Optimality below the one-sided bound is unverifiable; correctness is not certified.
4. Admissible coordinate magnitude differs per version (4.x NaN at 1e6 for general F; 5.0 at 1e100).
5. OpenCV libraries were not sanitizer-instrumented; polynomial coefficients/roots were not observed directly.
6. macOS and Windows native behaviour of the solver are unexamined.

---

## Revised decision: guarded-binding contract (Task 015, approved revision)

The **SAFETY_GATE_NOT_ESTABLISHED** disposition above remains correct for the
earlier, stronger requirement: reliable global numerical optimality for every
supported input. Nothing below retracts it; obligations 1–3 are still unmet.

The approved objective was then narrowed to a **guarded binding**: OpenCV performs
the correction, and the binding validates inputs, applies *caller-configured*
conditioning and quality screens, and verifies the output independently. Success
means only that those explicitly checked conditions held. See
[contract](epipolar-correction-contract.md) and
[validation](epipolar-correction-validation.md).

How each obligation is handled under the narrower contract:

1. Residual tolerance: chosen explicitly by the caller (`Maximum_Corrected_Error_Pixels`);
   no hidden epsilon. Not source-derived; documented as such.
2. Rank-2 cutoff: chosen explicitly by the caller (`Minimum_Second_Singular_Ratio`,
   `Maximum_Third_Singular_Ratio`). No universal cutoff is claimed.
3. Optimality below the bound: still unverifiable. The one-sided bound is retained
   strictly as a **necessary** condition.
4. Per-version coordinate limits: remain; native NaN/residual failures are rejected
   by the output gate, and version-specific dispositions are tested.
5. OpenCV libraries remain uninstrumented; actual-shim sanitizers only.
6. macOS native behaviour is exercised only by hosted CI; Windows remains unqualified.

### Re-verification (2026-10-13, before adding provenance)

* `/tmp/ocv/rev-*.txt` and `git rev-parse HEAD` of each source checkout equal the
  three pinned commits; every reviewed file is clean (`git status --porcelain` empty).
* SHA-256 and `git hash-object` of both triangulate.cpp and both headers per version
  were recomputed and equal `git rev-parse HEAD:<path>` and the table above /
  `docs/source-provenance.json` (headers were already recorded by earlier tasks).
* Run `38198320642`: `gh run view` reports workflow "Pinned OpenCV compatibility",
  head `06d1ac4953c19c17a37e2031b64fe7afffb796eb`, conclusion success for
  `native-version (4.1.0)`, `(4.10.0)` and `(5.0.0)`. Run `38197635569` (Windows
  post-merge): failure.
* The earlier `opt.py` output was not re-relied upon. Displacement-vs-bound facts used
  by this task were re-measured with the shim's own `correction_profile.hpp`
  (probe `/tmp/ocv/guard/{g,h}.cpp`) against all three source-built versions; 4.1.0 and
  4.10.0 output was byte-identical.

### Re-measured facts used by the guarded tests (normalized F)

| Fixture | 4.1.0 / 4.10.0 | 5.0.0 |
|---|---|---|
| rectified, 10 F scales 1e±300 … | (100,22)/(90,22), disp √8, bound 4 | same |
| rotated rank-1, pair (-30,55)/(12,70) | disp 54.27 vs bound 11.15 | same |
| s2/s1 = 1e-6, same pair | disp 12.78 vs 11.15 | disp 11.55 vs 11.15 |
| F=[e]×, 1e-9 from epipole | disp 0.170 vs 6.4e-10 | disp 6.4e-10 (within) |
| general F, coordinates 1e5…1e100 | NaN | finite; residual 0.61 at 1e15, NaN at 1e100 |
| exactly at epipole | undefined line (rejected pre-native) | same |
