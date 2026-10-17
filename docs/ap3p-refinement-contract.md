# Reference-validated AP3P pose refinement

`Select_Refine_AP3P_Pose` selects one AP3P hypothesis and then refines that
owned pose. It does not add an optimizer. Refinement is one call to the
existing `Refine_Pose_Iterative` binding, which uses native
`solvePnP(..., useExtrinsicGuess=true, flags=SOLVEPNP_ITERATIVE)`.

No new C ABI operation and no C++ change. The private ABI remains 48.

## Three observation sets

The caller supplies three logically distinct correspondence sets:

- **Seed:** exactly three world/pixel pairs. These are the only pairs passed
  to `Solve_AP3P_Hypotheses`.
- **Selection:** zero or more additional pairs. A nonempty set is passed to
  the existing Task 016 selector. Those same pairs, together with the three
  seeds, are the only pairs passed to iterative refinement.
- **Held-out:** one or more pairs. They are used only to evaluate the
  original selected pose and, after native success, the candidate refined
  pose. They are never supplied to AP3P, candidate selection, or refinement.

An empty selection set returns `No_Independent_References` and
`Selection_Only` before any solver or refinement call.

World points must be unique inside each set and must not repeat across sets.
Equality is exact component equality. There is no near-duplicate epsilon.
Pixel observations stay in their original paired order. Ada array bounds may
differ; pairing is positional.

Disjoint inputs are not statistical independence. Measurements can still
share calibration error, timing error, or another systematic bias.

Lengths, finiteness, and disjointness are checked before native entry.
A correspondence count that cannot be represented as a native `int` is
rejected. Refinement itself still requires at least four fitting pairs, so a
unique selection with no selection references is not refined.

## Acceptance policy

`AP3P_Refinement_Acceptance_Options` has no defaults. Every call supplies:

- `Maximum_Holdout_RMS_Pixels`, inclusive, finite, nonnegative;
- `Maximum_Holdout_Point_Error_Pixels`, inclusive, finite, nonnegative;
- `Maximum_Holdout_RMS_Worsening_Pixels`, finite and nonnegative.

Acceptance uses the whole held-out set. It does not use the fitting residual.
The refined result is accepted only when all of the following hold:

- native refinement returned true;
- every held-out reference is finite and at positive camera depth;
- held-out RMS is at or below the caller's maximum;
- the maximum held-out pixel error is at or below the caller's maximum;
- held-out RMS does not worsen by more than the caller's allowance.

Worsening is the direct subtraction `refined RMS - original RMS`. A
nonpositive change always satisfies a nonnegative allowance. When
`0 <= original < refined <= Float64'Last`, that subtraction is representable
and is not scaled, clamped, or compared through an epsilon. The comparison is
inclusive. A nonrepresentable input raises `OpenCV_Error` and publishes nothing.

Native success alone does not mean the pose is better or accepted.

## Dispositions

| Disposition | Meaning |
|---|---|
| `Selection_Only` | Task 016 did not return a unique owned pose. Refinement is not called. |
| `Initial_Holdout_Unusable` | Selection succeeded, but the held-out set is not entirely positive-depth and evaluable. |
| `Native_Refinement_False` | Native iterative refinement returned false. The original pose is unchanged. |
| `Refined_Holdout_Unusable` | Native refinement succeeded, but the candidate cannot be fully evaluated on the held-out set. |
| `Rejected_By_Policy` | Both held-out summaries are valid, and the caller policy rejects the candidate. |
| `Accepted` | Every check succeeded and the refined pose is published. |

`Initial_Pose` is available whenever selection uniquely owned a pose,
including every nonacceptance disposition after that point. `Accepted_Pose`
raises `OpenCV_Error` unless the disposition is `Accepted`. An unaccepted
native pose is not retained in the result.

Held-out summaries are available only for the evaluations that completed.
A missing summary raises `OpenCV_Error`; a stored zero is not implied.

`OpenCV_Error` from validation, projection, AP3P, or refinement propagates.
No partial result is published. A native exception does not cross the C ABI.

## Frame

`X_camera = R * X_world + t`. `Translation` is `t`, not the camera position.
`Camera_Center` remains `C_world = -R^T * t`. The world frame and its units
are the caller's Cartesian frame. This operation does not produce an Earth
frame, a DTED coordinate, a covariance, a position uncertainty, or a claim of
metric navigation accuracy.

Rotation agreement is established with `Rotation_Matrix_Of`, not by comparing
Rodrigues components. A unique or accepted result is conditional on the
supplied observations and the caller's numerical policy.
