# AP3P reference-diagnostic qualification

Local OpenCV 4.10.0 / calib3d. Hosted CI and the Task 013 pinned matrix are
not claimed here.

## Task 012 baselines

No Task 012 pinned matrix existed before this task. One manual dispatch on
main `604458fcf11a56ff94dfccc46badccdde283e847` is run `38190879037`.

```text
OpenCV 4.10.0 / calib3d   PASS
OpenCV 5.0.0 / geometry   PASS
OpenCV 4.1.0 / calib3d    FAIL
```

The 4.1 failure is in the existing AP3P shim, at the two assignments into
`cv::Mat::at<cv::Vec3d>` and `cv::Vec2d`. GCC reports deprecated implicit
copy assignment with `-Werror=deprecated-copy`. It is not a Task 013 numeric
defect. The assignments now write vector components through references.
That correction is included in this branch and still needs the Task 013
pinned matrix. `WITH_ADE=OFF` was retained. The run was not repeated.

Task 012 Windows post-merge run `38189547375` failed before compilation
during MSYS2 installation. `libgcc` / `libstdc++` `16.2.0-5` conflicted with
packages requiring `16.2.0-4`. No binding was compiled. Windows CI is
unchanged and remains post-merge only.

## Local measurements

107 AUnit registrations, 107 executed, 107 passed, 0 failed assertions,
0 unexpected errors. All 99 Task 012 registrations remain. Eight new cases
cover the known fourth reference, five independent references, non-1 bounds,
strict depth, five-coefficient distortion, an extreme finite residual,
summaries, and empty or malformed input.

Known fourth reference, two hypotheses:

```text
known-candidate error 2.84217094304040e-13 pixels
camera depth         6.29014508315020
```

The alternate hypothesis is retained. Its error is not a cross-version
constant. Five additional references give the known candidate RMS
`8.13873899873917e-14` and maximum `1.27105748646260e-13`. The alternate has
RMS `1.46776468841858e2` and maximum `2.46632045684130e2`. Nothing selects
a pose.

Identity depth classifies `Z=5`, exact `Z=0`, and `Z=-2`. Translation and
`Rz(pi/2)` use `R` and `t`, not `R^T` or the camera center. Five nonzero
distortion coefficients agree with a test-only Brown equation and with
`Project_Points` / `Reprojection_Errors`. Production Ada does not contain
that polynomial.

A finite native prediction compared with an observed pixel of `-1e308`
remains a finite projected error. Float64 can represent that subtraction,
so it is not `Numerically_Unavailable`. A native `Project_Points` failure
still propagates as `OpenCV_Error`.

A real `Numerically_Unavailable` disposition is executed through
`Evaluate_AP3P_References`: pose `Rz(pi/4)`, zero translation, and world
points `(0,0,4)` and `(1.5e308,-1.5e308,4)`. The camera X sum of the two
rotated products (each about `1.06e308`) exceeds Float64, so the second
reference returns `Numerically_Unavailable` with stored zeros while the
first stays `Projected_Positive_Depth`. The summary is Projected=1,
Unavailable=1, Non-positive=0, Total=2, not `Entire_Set_Evaluable`, with
`Reprojection.Count=1`. This summary comes from the evaluator, not a
hand-built record. The residual-subtraction overflow path is not claimed:
no portable finite native prediction near `Float64'Last` was established.

The fault runner reuses projection checkpoints 1 and 2, five categories
each. All 10 raise `OpenCV_Error`. A zero-depth reference does not enter
projection. The private ABI remains 46 imports and exports.

Python configuration and repository checks: 24 passed. Profile helpers,
shell syntax, `alr test`, raw ABI, and actual-shim production and fault
ASan/UBSan passed. All five examples passed. The PnP example prints both
diagnostic rows and does not name a winner.

Each positive-depth reference is projected separately. Empty matrices do
not call `projectPoints`.
