# Physical plane third-view verification (Task 030)

`Verify_Physical_Plane_Third_View` consumes the complete owned Task 029 result.
It never repeats homography fitting, reciprocal verification or intersection.
The supplied third camera verifies only; it cannot modify the source point.

## State precedence and trust boundary

The first applicable per-observation disposition wins:

1. `Prior_Heldout_Rejected`: the individual Task 029 held-out result rejected.
2. `Prior_Model_Rejected`: the complete Task 029 aggregate rejected.
3. `Plane_Third_Non_Positive_Depth`: evaluated third depth is zero or negative.
4. `Plane_Third_Numerically_Unavailable`: evaluator arithmetic is unavailable.
5. `Plane_Third_Reprojection_Rejected`: projected error exceeds the explicit limit.
6. `Physical_Plane_Third_View_Accepted`: every preceding gate passed.

Only individually accepted Task 029 held-outs are eligible. Their world positions
come exactly from `Accepted_First_Source_World_Position (Physical_Verification
(Heldout))`, preserving Task 026's first-camera representative, never its midpoint
or an unaccepted diagnostic intersection. Eligible points are evaluated even if
the prior aggregate rejects, preserving independent diagnostics without repairing
the rejected model. Final individual acceptance is exactly prior aggregate AND
prior individual acceptance AND positive-depth projection AND error within limit.
Aggregate acceptance additionally requires every original held-out to accept.

## Validation, execution and units

Before native execution, require matching third-pixel/original-heldout counts that
fit INT32, every finite pixel (including rejected candidates), finite positive
focal lengths, finite principal point, all five finite Brown coefficients, all six
finite pose components, and finite nonnegative `Maximum_Third_Pixel_Error`.
Malformed requests raise `OpenCV_Error`. Pixel pairing uses iteration position,
not source subscripts; unrelated positive lower bounds and arrays ending at
`Positive'Last` are supported. Output positions are original one-based held-out
correspondence positions, never compacted identities.

All poses share the caller's Cartesian world frame and length unit:

```text
X_camera = R * X_world + t
C_world = -R^T * t
```

Rotation is Rodrigues; translation is t, not camera position. Depth and world
coordinates use caller world units. Third and later observations are raw
Brown-distorted pixels. Third errors/thresholds are pixels, not Task 028's
dimensionless normalized coordinates. Acceptance is directly inclusive `<=`,
without epsilon, minimum depth, confidence probability or implicit defaults.

Owned compact world/pixel arrays and an explicit compact-to-original mapping
enter `Evaluate_AP3P_References` at most once, with exactly one supplied pose.
Zero eligible points skip it entirely. For a nonempty batch, the evaluator uses
native Rodrigues and zero or one native projection batch, depending on positive
depth. No per-point projection or independent production Brown implementation is
introduced. The actual `AP3P_Reference_Disposition` is retained unchanged.

## Ownership and guarded diagnostics

The private aggregate's discriminants constrain its complete prior evidence and
one diagnostic per original held-out. All data are owned Ada values, independent
of input mutation/finalization and native temporary lifetimes. Core still owns
Mats; existing callback-scoped borrowing is unchanged. Production C++ and the
private C ABI remain unchanged at 48 operations.

`Prior_Evidence`, `Holdouts`, `Holdout_Count`, `Accepted`, `Original_Position`,
`Prior_Heldout_Accepted`, `Third_View_Evaluated` and `Disposition` are unguarded.
`Selected_World_Position` requires prior individual acceptance and is diagnostic,
not final acceptance. `Third_View_Disposition` requires evaluation;
`Third_Camera_Depth` additionally requires a non-numerically-unavailable evaluation
(retaining zero/negative depth). `Third_Pixel_Error` requires positive-depth
projection. `Accepted_World_Position` requires final individual acceptance.
Absent guarded values raise `OpenCV_Error`, never expose storage zeros as metrics
or points. Default per-observation records are rejected with no diagnostics.

Native exceptions and invalid native output propagate as `OpenCV_Error`, with no
partial return; failed assignment preserves an existing complete value. Evaluator
per-point numerical unavailability is a diagnostic rejection, not a native error.

## Accepted-only handoff and limits

`Physical_Plane_Third_View_Landmark_Observations` validates the entire later-pixel
batch and exact original count, even for unused observations. If the prior
aggregate rejected, it returns zero landmarks. Otherwise it compacts individually
final-accepted points, preserving world positions, corresponding supplied later
pixels, original positions and original order. A partially rejected third-view
aggregate does not erase valid survivors. Empty arrays have bounds `1 .. 0`.
The later camera's pose is neither estimated nor verified.

All landmarks may be coplanar. Landmark count alone does not establish sufficient
depth diversity, PnP uniqueness, general six-degree-of-freedom observability,
robust relocalization or metric navigation accuracy. Third-camera agreement is
compatibility with supplied models, not physical ground truth, correct feature
identity, statistical independence or an established uncertainty bound. Shared
systematic pose, plane, calibration and matching errors remain possible.

No DTED/DEM reading, terrain selection, geodesy, pose estimation, bundle adjustment,
visual odometry, SLAM, feature matching or navigation fusion is included.