# Robust landmark-based camera relocalization (Task 021)

`Relocalize_Camera` composes existing native fixed-EPNP `Solve_PnP_RANSAC`,
one `Refine_Pose_Iterative` call, and independent held-out diagnostics. No solver,
optimizer, dependency, C++ code or private ABI operation is added (48 remain).
The refinement is `solvePnP(SOLVEPNP_ITERATIVE,useExtrinsicGuess=true)`, **not**
`solvePnPRefineLM`. Core continues to own every temporary Mat; borrowed native
headers remain callback scoped. Results contain only owned Ada values.

## Inputs and isolation

Fitting and held-out arrays are paired by position regardless of lower bounds.
Each world/pixel count must match, fit native INT32, and contain finite Cartesian
world coordinates and finite pixels. Each world set must have no exact repeated
point and the two sets must have no exact world-point overlap. Equality is exact
component equality; there is no near-duplicate threshold. At least one held-out
pair is mandatory. Calibration and all five Brown coefficients use the existing
validation contracts. All options and observations are validated **before** the
insufficient-fitting return, even for empty fitting arrays.

At least **six fitting pairs** are required for estimation. Valid requests with
zero through five pairs return `Insufficient_Fitting_Landmarks` without native
entry. This avoids OpenCV's special four-point direct P3P and five-point direct
EPNP paths. Robust fitting otherwise retains existing iteration, reprojection
threshold/native Float32 conversion, and confidence semantics.

Held-out points never enter RANSAC, inlier compaction, or iterative refinement.
Disjoint landmarks do not guarantee statistical independence: reconstruction,
calibration and other systematic errors may be shared. Both sets must already
share the caller's Cartesian frame and length units.

## Consensus, refinement and ownership

Native `Found=False` returns `No_Native_RANSAC_Pose`, not an exception. A found
pose and its sorted one-based native consensus positions are copied out of the
limited-controlled estimate. The returned private discriminated value remains
usable after that estimate finalizes. `Count`/`Inlier_Count` is the **native**
consensus count; `Inliers` maps to original fitting-correspondence positions,
not Ada subscripts. No observation is reordered before estimation.

The caller's minimum support must be at least four. A found model below that
minimum returns `Insufficient_Consensus_Inliers`. Otherwise initial held-out
verification precedes refinement. If usable, compact exactly the native inlier
pairs, initialize a local pose from the copied RANSAC pose, and refine once.
Rejected fitting observations and held-out observations are never included.
Native false retains the original pose and valid initial held-out summary.

Consensus identities describe the final native RANSAC model **before** subsequent
refinement. They are not automatically reclassified inliers of the refined pose.
`Final_Inlier_Errors` diagnoses those same identities in their sorted order;
`Final_Inlier_Summary` reports their whole-set RMS and maximum.

## Whole-set verification and explicit policy

The shared existing evaluator obtains native `Rotation_Matrix_Of`, checks finite
`R*X+t` coordinates and strictly positive camera Z (no epsilon), and uses native
`Project_Points` with the supplied intrinsics/distortion. Residuals are Euclidean
pixel distances; scaled norms and `Summarize_Reprojection` avoid unnecessary
overflow. Any nonpositive-depth or unavailable arithmetic observation makes the
**whole set** unusable. No partial-set RMS is exposed as a whole-set score.
Native projection/rotation exceptions propagate as `OpenCV_Error`, not routine
unavailability. The initial and refined held-out summaries are separately guarded.

All five verification fields are mandatory. The four pixel limits must be finite
and nonnegative. Acceptance requires native pose, adequate support, native
refinement true, positive-depth fully evaluable final consensus and held-out sets,
and these inclusive comparisons:

* final consensus RMS <= `Maximum_Final_Inlier_RMS_Pixels`;
* refined held-out RMS <= `Maximum_Holdout_RMS_Pixels`;
* refined held-out maximum <= `Maximum_Holdout_Point_Error_Pixels`;
* positive held-out RMS worsening <= `Maximum_Holdout_RMS_Worsening_Pixels`.

The existing Task 017 policy uses direct subtraction:

```ada
if Refined_RMS <= Initial_RMS then
   return True;
end if;
return Refined_RMS - Initial_RMS <= Allowance;
```

No scaled subtraction, hidden tolerance, clamp or empirical threshold is used.
Native optimizer success alone never publishes an accepted pose.

## Dispositions and guards

| Disposition | Meaning |
|---|---|
| `Insufficient_Fitting_Landmarks` | Valid request has fewer than six fitting pairs. |
| `No_Native_RANSAC_Pose` | Expected native false, no pose or inliers. |
| `Insufficient_Consensus_Inliers` | Pose retained, caller minimum not met. |
| `Initial_Holdout_Unusable` | Initial whole held-out set cannot be scored; no refinement. |
| `Native_Refinement_False` | Attempted once, returned false; initial pose retained. |
| `Refined_Holdout_Unusable` | Native true but whole refined held-out set unusable. |
| `Final_Inlier_Verification_Unusable` | Refined holdout usable, final native-consensus set unusable. |
| `Rejected_By_Policy` | Valid scores fail at least one caller limit. |
| `Accepted` | Every check passed; pose and center are published. |

`Found`, `Refinement_Attempted`, `Native_Refinement_Succeeded`,
`Initial_Holdout_Valid`, `Refined_Holdout_Valid`, `Final_Inliers_Valid` and
`Accepted` describe availability. `Initial_Pose` requires `Found`.
Both held-out summaries and final inlier diagnostics raise `OpenCV_Error` when
unavailable; internal zero initialization is not a measured score. Meaningful
available summaries survive rejection, including final inlier diagnostics when
only refined holdout is unusable. `Accepted_Pose` and `Camera_Center` raise on
**every** nonaccepted disposition. All fallible accepted values are staged before
acceptance. Validation/native errors propagate without publishing a partial result.

## Task 020 adapter

`Accepted_Landmark_Observations` requires matched landmark/new-pixel counts and
finite pixels. It reads coordinates only through `Accepted_World_Position` for
values where `Accepted=True`. No unavailable original or unaccepted candidate
geometry is trusted. `World_Points`, `Image_Points` and `Original_Positions` are
compacted `1..Count` owned arrays; pairing is positional even at `Positive'Last`.
Nothing qualifying returns `1..0`. Primary relocalization accepts ordinary arrays
and does not require this adapter. To map relocalization consensus to the original
Task 020 array, use `Original_Positions(Set)(Inliers(Result)(I))`.

Task 020 acceptance is reprojection-policy acceptance, not proof that a landmark
is physically correct. The adapter adds no such guarantee.

## Frames and limitations

`X_camera = R * X_world + t`; translation is t, **not camera world position**.
`C_world = -R^T * t`. Length units come solely from supplied trusted landmarks.
No geographic reference, DTED interpretation, frame conversion, covariance,
uncertainty propagation, SLAM, bundle adjustment, Kalman filtering, terrain
matching or navigation fusion is supplied.

Successful held-out verification establishes compatibility only with the supplied
landmarks, observations, calibration, and caller policy. It is not independent
physical ground truth, globally correct localization, a statistical uncertainty
bound or evidence of real navigation accuracy.