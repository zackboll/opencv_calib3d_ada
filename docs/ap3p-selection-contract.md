# Reference-qualified AP3P pose selection

`Select_AP3P_Hypothesis` qualifies hypotheses that `Evaluate_AP3P_References`
has already scored. `Select_AP3P_Pose` is the end-to-end convenience
operation: exactly three seed correspondences, additional references, the
existing `Solve_AP3P_Hypotheses` wrapper, evaluation, then the same policy.

No new C ABI operation and no C++ change. The private ABI remains 48.

## Policy

`AP3P_Selection_Options` has no defaults. Every call supplies:

- `Maximum_RMS_Error_Pixels`, inclusive, finite, nonnegative;
- `Maximum_Point_Error_Pixels`, inclusive, finite, nonnegative;
- `Minimum_RMS_Separation_Pixels`, finite and nonnegative.

A candidate is eligible only when `Entire_Set_Evaluable` is true and both
the whole-set RMS and the whole-set maximum are within those thresholds.
That requires at least one reference and `Projected_Positive_Depth` on every
reference. `Non_Positive_Depth` and `Numerically_Unavailable` reject the
candidate. RMS is never computed from the successful subset and then treated
as a whole-set score.

Eligible candidates are ranked by RMS. Original order breaks exact RMS ties.
Rank is diagnostic. With one eligible candidate the result is unique. With
two or more, `second_best - best` must be strictly greater than
`Minimum_RMS_Separation_Pixels`. An equal gap, an exact tie, or a
nonrepresentable difference is not a unique selection. There is no implicit
epsilon and no clamping.

## Dispositions

| Disposition | Meaning |
|---|---|
| `No_Hypotheses` | The solver or the supplied matrix has no hypothesis. |
| `No_Independent_References` | No additional reference was supplied. |
| `No_Eligible_Hypotheses` | References exist, but none meet the policy. |
| `Ambiguous_Qualified_Hypotheses` | At least two candidates are eligible and not strictly separated. |
| `Unique_Selected_Hypothesis` | Exactly one candidate survives the policy. |

`Selected` is true only for the last disposition. `Selected_Hypothesis_Index`
and `Selected_Pose` raise `OpenCV_Error` otherwise. The pose index is the
hypothesis array's own index; selection identity is position, not a fixed
native candidate number and not a Rodrigues-component comparison.

## End-to-end ownership

`Select_AP3P_Pose` returns `AP3P_Pose_Selection`. That private type copies the
selected `World_To_Camera_Pose` from the one `Solve_AP3P_Hypotheses` call it
made. `Selected_Pose` on that type takes no hypothesis array and no index.
It raises `OpenCV_Error` unless the disposition is unique and the owned pose
is present. Callers cannot assemble this type.

`AP3P_Selection_Result` remains the matrix-only result. Its `Selected_Pose`
checks the supplied array's length and position only. It does not prove that
the array is the one that was evaluated. Substituting another array of the
same length returns that array's pose, not a provenance-checked pose.

## End-to-end validation

`Select_AP3P_Pose` rejects every seed count other than three, mismatched
reference counts, and nonfinite inputs. Reference world points must differ
exactly from every seed world point and from each other. There is no
near-duplicate tolerance. Arrays of any valid lower bound are paired by
position.

A legitimate native zero-hypothesis result returns `No_Hypotheses`. An empty
reference set returns `No_Independent_References` before the solver.
`OpenCV_Error` from `Project_Points` or from the solver propagates and no
selection value is published.

## Frame

`X_camera = R * X_world + t`. `Translation` is `t`, not the camera position.
`Camera_Center` remains `C_world = -R^T * t`. Units are the caller's
Cartesian world units. No geographic coordinate, DTED conversion, covariance,
or uncertainty estimate is produced.

A unique result is unique only for the supplied references and the supplied
policy. It is not proof of globally correct camera localization.
