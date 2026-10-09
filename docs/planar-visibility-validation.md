# Task 011 qualification record

Starting main: `471da8f538a637870ef710161b6005a8478c1b28`.
Branch: `feature/011-homography-visibility`. Version remains `0.1.0-dev` and
Core pin remains `4da9d35ea21e1b2efe96296243ea668b488c6326`.

## Task 010 post-merge baseline

Exact Windows run [37876994035](https://github.com/zackboll/opencv_calib3d_ada/actions/runs/37876994035)
completed successfully at the starting main. Its actual log records OpenCV 5.0.0
geometry, MSYS2 MinGW64 `g++.exe` under Alire's MSYS2 cache, 83 executed/83 passed
AUnit cases, DLL/import-library existence qualification, and PE imports
`libopencv_geometry-500.dll` and `libopencv_core_shim.dll`.
Exact main-push run [37876993922](https://github.com/zackboll/opencv_calib3d_ada/actions/runs/37876993922)
also completed successfully: repository-checks, linux, macos, linux-sanitizers.
This is Task 010's Windows baseline only; Task 011 Windows remains post-merge-only.

## Inventory and evidence status

Task 010's 83 registrations are preserved; ten separate visibility registrations
bring the inventory to 93. Exact C/Ada declaration/import set comparison gives
45 private operations (one new pair). Independent fixtures cover normal sign,
strict zero, nonidentity R*n versus R^T*n, decomposition composition, selected
robust-homography inliers, pure/near rotation, empty/lower bounds, invalid records,
Float32 range/underflow and compiler-derived result layout.

## Local executed results (OpenCV 4.10.0 / calib3d)

* AUnit: 93 registered/executed/passed, zero failed assertions/unexpected errors.
* Repository static check: matching 45-operation ABI, manifests/pins/ownership and
  unchanged four-job CI topology PASS. Python configuration/static: 24/24 PASS.
* Six C/C++ profile helpers PASS; syntax checks on all seven shell scripts PASS.
* Root/tests/example builds and `alr test` PASS, warnings remain errors.
* Actual-shim native production and fault drivers PASS; new raw faults 20/20,
  Ada exception translation 20/20, plus five deliberately malformed returned-index
  controls after real filtering. Raw output remains cleared on every failure.
* Armed checkpoint proves pure/no-reference Ada branches skip native entry.
* Actual modified shim production/fault ASan+UBSan PASS with leak detection and
  halt-on-error, no suppressions. Linked Core/OpenCV are not claimed fully instrumented.
* All five examples PASS: PnP, homography, fundamental, Essential, triangulation.
* New result C sizeof/alignment=8/4; Ada Size=64 bits and type Alignment=4;
  count/array offsets=0/4; accepted elements=4,5,6,7. Full C-written/Ada-read
  interchange PASS. GNAT may align a standalone object more strongly (observed 8),
  which does not change the record ABI. Existing Task 010 layouts retained/PASS.

## Independent fixture findings

Normal sign: (+1,+1) survives, (-1,-1) rejects; Ada passing positions `[1]`.
Strict boundary: first signs 0,+0.25,-0.25 with second +0.25 give rejected,
passing,rejected. Nonidentity Y rotation 0.2, n=(0.6,0,0.8), p1=(0,0,1),
p2=(-1.2,0,1): first=0.8; correct R*n second=-0.231518829812996 while incorrect
R^T*n second=+0.388329482267595. Native rejects; independent oracle includes
Float32 rounding of coordinates.

Task 010 composition on this native version: passing positions `[1,3]`, rejected
`[2,4]`; multiple surviving motions are valid. Robust homography selected-inlier
fixture: 24 supplied, 21 selected; gross outliers 2,7,15 excluded; controlled
candidate rejects with all points but passes selected constraints.
Native compact/no-mask versus full/CV_8U-mask comparison gives exactly `[0,2]`
(zero-based), with first/second signs +0.2/+0.25 for selected positive normals.
Pure and near rotation are not applicable, with no passing indices.
Native zero survivors is successful. Conversion tests reject overflow and
nonzero-to-zero underflow and accept finite subnormal/large Float32 values.

Ordinary PR CI and pinned compatibility matrix results are pending execution;
local evidence does not establish those gates.