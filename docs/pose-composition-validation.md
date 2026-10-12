# Pose composition qualification

Local OpenCV 4.10.0 / calib3d on Linux. Hosted CI and the Task 014 pinned
matrix are **not** claimed here; they are reported separately in the pull request.

## Task 013 baseline

Main `f1f79687ffd843cfc0e239ff95c71bc0050f34c9` (PR #13).

* Cross-platform main push `38193834861`: success.
* Windows post-merge `38193834911`: **failure**, before compilation, in the MSYS2
  package transaction (`libgcc 16.2.0-5` would break `cc-libs` requiring
  `libgcc=16.2.0-4`). A package-manager failure, not a binding failure. The
  Windows workflow is unchanged; Windows remains unqualified.
* No pinned run existed for this SHA. One manual dispatch, run `38194995507`,
  checked out exactly `f1f7968...` and passed on 4.1.0, 4.10.0 and 5.0.0
  (107/107 AUnit each, 0 unexpected errors; `WITH_ADE=OFF`). This is the first
  execution of the corrected AP3P `cv::Vec` path on OpenCV 4.1. It is baseline
  evidence only and is not evidence for Task 014.

## Local results

```text
Production build                    PASS
AUnit registrations                 115 (107 + 8 new); run 115, passed 115,
                                    failed assertions 0, unexpected errors 0
alr test                            PASS
Repository check                    PASS: 47 ABI declarations/imports, 115 registrations
Python configuration tests          24 PASS
Profile helpers                     PASS (all)
Shell syntax checks                 PASS
Raw native boundary (production)    PASS, incl. compose order/identity/rejections/aliasing
Raw fault injection                 20 scenarios (checkpoints 54..57 x 5 kinds)
Ada exception translation           20/20 translated, recovery after each
ASan+UBSan production shim          PASS
ASan+UBSan fault-injection shim     PASS
Production binary leak check        PASS (no opencv_calib3d_test_ symbol)
Examples                            all five exit 0
git diff --check                    PASS
```

## Tolerances and measurements

* Composition vs independent oracle (R2*R1, R2*t1+t2): rotation matrix 1e-12, translation 1e-12.
* Sequential vs composed point transform: 1e-12. Example maximum difference 1.86e-15.
* Native projection (zero and five-coefficient Brown) vs scalar sequential oracle:
  threshold 1e-8 pixels; measured maximum 2.27e-13 pixels.
* Rodrigues boundary cases: matrices compared, not vectors; tolerance 1e-5
  (admits the native flush of angles below about 1e-5 to zero).
* Layout: compiler-derived C `sizeof`/alignment equal Ada `C_Pose'Size`/`'Alignment`;
  all six fields interchange with distinct values; inputs unchanged after calls.

## Observed native behavior

* Squared-norm overflow of a finite rotation vector silently produced a zero
  rotation or NaN translation natively; the shim rejects it up front.
* Composed rotation of physical angle 2e-9 (pi-1e-9 twice) is returned as an
  exact zero vector natively; the matrix error is far below 1e-8.
* Translation cancellation at +/-Float64'Last gives exactly zero; two
  maximal translations overflow and are rejected.
* Alias calls (`result == first/second/both`) return a nonzero status and leave
  the input record unchanged.

## Not established here

Hosted CI, OpenCV 4.1.0 and 5.0.0 execution of the new tests, and Windows are
not claimed by this document. The private ABI is 47 operations.
