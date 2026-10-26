# Physically qualified plane-homography verification (Task 029)

`OpenCV.Calib3D.Verify_Physical_Plane_Homography_Consistency` combines independent
algebraic and physical compatibility evidence. It neither estimates supplied
geometry nor establishes ground truth.

## Inputs, frames, units and identity

The signature uses existing `Image_Point_Array`, `Camera_Intrinsics`,
`Distortion_Coefficients`, `World_To_Camera_Pose` and `World_Plane` types. The four
pixel arrays are separate fitting-first, fitting-second, held-out-first and
held-out-second batches. Pairing is positional **within** each batch; fitting and
held-out observations are never combined. At least five fitting pairs and one
held-out pair are required, with each count fitting INT32. Lower bounds may differ,
including arrays ending at `Positive'Last`.

Each camera has independent positive focal lengths, principal point and five Brown
coefficients. Inputs are raw Brown-distorted pixels, not ideal pinhole pixels.
Both complete world-to-camera poses obey:

```text
X_camera = R * X_world + t
C_world = -R^T * t
```

Rotation is a Rodrigues vector; translation `t` is not camera location. Both poses
and the infinite Cartesian plane share one world frame and caller length unit.
No geographic meaning, meters conversion, or Earth model is assumed.

Both policy records are mandatory, with no new defaults:

* `Plane_Homography_Consistency_Options`: RANSAC and three Euclidean comparison
  thresholds are **dimensionless normalized camera coordinates**, not pixels.
* `Reciprocal_Plane_Verification_Options`: independent forward/backward Task 025
  policies. Reprojection limits are **pixels**; incidence cosine is dimensionless;
  range, represented-plane residual, depth and world separation use **caller world
  length units**. `Maximum_World_Separation` is finite and nonnegative.

All underlying Task 026/028 policies, inclusive comparisons and numerical
availability rules are unchanged. No undocumented conversion connects these units.
See [Task 028](plane-homography-consistency-contract.md),
[Task 026](reciprocal-plane-correspondence-contract.md),
[Task 025](plane-correspondence-contract.md) and
[Task 024](world-plane-intersection-contract.md).

Outputs use original **one-based correspondence positions**, not Ada caller
subscripts. Fitting inlier positions remain within the fitting batch. Held-out
positions remain within the held-out batch in the combined, algebraic and physical
records. Copies and slices preserve their stored identities. The caller remains
responsible for feature identity, separating observations, sampling independence
and provenance; coordinate agreement cannot establish these properties.

## Complete pre-native validation and execution ordering

The narrow shared private `Validate_Consistency_Request` checks all four counts
and arrays, every pixel including late rejected observations, both calibrations,
both Brown models, every pose component, finite plane anchor/nonzero finite normal,
RANSAC options, final support bounds and all three finite nonnegative limits.
The shared reciprocal policy validator checks the plane and both complete
directional policies plus the separation limit. These validators contain no native
operations. The original Task 026/028 operations reuse them, retaining validation
conditions, ordering and error messages.

Only after the **entire** integrated request is valid:

1. Call `Verify_Known_Plane_Homography_Consistency` exactly once on the two separate
   batches. It constructs the prediction once, and fits empirical H once when the
   predicted model is defined. Its existing undefined-prediction early return is
   preserved: an empirical fit is not invented for unavailable geometry.
2. Call `Verify_Reciprocal_Plane_Correspondences` exactly once on held-out pixels,
   using all original camera/plane arguments and the explicit physical policy.
   This happens even for absent prediction, no empirical model, insufficient
   fitting support, or algebraic residual rejection. Physical diagnostics are not
   conditioned on empirical success. Task 026 still delegates once per direction.
3. Combine the complete results in Ada. No native call occurs inside the combining
   loop. No second fit, held-out fitting, refinement or alternate estimator exists.

Malformed inputs and native failures raise `OpenCV_Error`, with **no partial
return**. No exception handler surrounds either verifier. A failed native operation
cannot be converted into numerical unavailability or a rejected partial result.
Assignment after such failure leaves an earlier complete destination unchanged.
Ordinary Ada constrained-assignment rules still apply: a variable initialized with
an indefinite result becomes constrained to that result's discriminants; do not
assign a healthy result with different counts to it. Use a fresh constant for
requests whose final inlier counts differ. Resource exhaustion is not guaranteed
to be translated to `OpenCV_Error` by Ada allocation/runtime operations.

## Owned result representation and diagnostic guards

`Physical_Plane_Homography_Result (<>)` is an indefinite private Ada value. Its
private `Fitting_Count` (actual final inlier count) and `Heldout_Count`
discriminants constrain the nested Task 028 result and held-out array directly.
It does not embed an unconstrained indefinite component or use heap pointers.

* `Algebraic_Consistency` returns the complete owned Task 028 result, including
  construction disposition, empirical model availability, owned matrix and final
  fitting inliers, all held-out targets, availability flags, residuals and states.
* `Holdout_Count` and `Holdouts` provide the full one-based held-out batch.
* Every `Physical_Plane_Holdout_Result` has `Original_Position`,
  `Diagnostics_Evaluated`, `Algebraic_Verification`, `Physical_Verification`,
  `Disposition` and `Accepted`. The two verification accessors return complete
  owned Task 028/026 diagnostics without stripping rejected evidence.
* A default held-out record is not evaluated or accepted. Reading either nested
  verification before evaluation raises `OpenCV_Error`.
* Every returned record has evaluated diagnostics, but individual values can be
  unavailable. The original nested guards remain authoritative: absent empirical
  H, targets, residuals, source intersections, target depth/projection or reciprocal
  separation must not be read without their predicates. Unavailable reads raise
  `OpenCV_Error`, never publish placeholder zeros as evidence.

Returned copies survive destruction/mutation of input arrays and native
temporaries. No borrowed handle, unchecked access value, C++/STL type or public raw
pointer is introduced. Core remains the sole Mat owner, with callback-scoped
Module_Interop borrowing in reused operations. Larger batches have proportionate
owned storage and copy costs; no unbounded physical accuracy claim follows.

## Acceptance and deterministic precedence

An individual held-out record is accepted **if and only if**:

1. Its known-plane prediction is a valid finite transfer.
2. Its empirical transfer is valid and finite.
3. All three Task 028 residuals are available and within their explicit normalized
   limits: predicted/observed, empirical/observed, model/model separation.
4. Its forward Task 025 physical verification passes, including forward ray/plane
   acceptance, positive target depth and distorted-pixel reprojection policy.
5. Its reverse Task 025 physical verification passes independently.
6. Both independently derived source positions are available, their separation is
   numerically available and it is within the explicit world-unit limit.

Per-point summary precedence is:

1. `Algebraic_Consistency_Rejected` when the Task 028 **held-out** result rejects.
2. `Physical_Verification_Rejected` when the held-out algebraic result accepts but
   its Task 026 reciprocal verification rejects.
3. `Physical_Plane_Homography_Accepted` otherwise.

Aggregate precedence uses the same enum, but first tests the **complete aggregate**
Task 028 result, then whether any held-out Task 026 result rejects. Thus aggregate
acceptance requires sufficient final fitting support and accepted algebraic result
**and every held-out physical verification**. An individual may pass both tests
while aggregate support rejects. Every independent diagnostic is computed before
summary precedence; a rejection never erases an accepted neighbor or hidden-stage
diagnostic. Inspect the nested dispositions to distinguish the exact cause.

All comparisons inherit inclusive `<=` semantics; no new epsilon, depth cutoff,
baseline heuristic, confidence probability, rescaling or weighting is added.

## Special cases and numerical limitations

* General plane-induced motion retains Task 027's complete construction result.
* Exact common-center pure rotation gives an algebraic mapping independent of the
  plane. It passes this integration only if the supplied plane actually supplies
  acceptable forward intersections and every physical condition passes. A plane
  through the common center is not physical terrain evidence. Pure rotation is
  neither automatically accepted nor automatically rejected by a baseline rule.
* A plane through either distinct camera center leaves prediction undefined; its
  exact Task 027 disposition survives and Task 026 still runs.
* Non-positive source range or target depth, parallel/coplanar rays, range/incidence
  rejection, numerical intersection absence, and reciprocal world disagreement
  retain the distinct Task 024/025/026 states and guards.
* No empirical model, insufficient final support, zero projective denominator,
  unrepresentable mappings, observations or residual arithmetic retain Task 028
  dispositions. Large finite input does not guarantee available arithmetic.
* Existing conservative checked matrix arithmetic, scaled norms and native Float32
  homography fitting limitations remain. Brown inversion is iterative and
  approximate. Moderate synthetic tolerances are fixture qualifications, not
  universal conditioning or accuracy guarantees.

The two physical intersections are independently derived but may share systematic
errors in poses/calibration/plane. They are never averaged, moved or refined.
Mutual agreement is **compatibility with the supplied models and thresholds**, not
certified terrain truth, statistical confidence, pose accuracy or navigation
accuracy. There is no DTED/DEM parsing, geodesy, image warping, matching, pose
estimation, plane estimation, uncertainty model or navigation solution here.

## Synthetic example

The seventeenth file-free example, at
`/home/zboll/git/opencv/calib3d/examples/src/physical_plane_homography_synthetic.adb`,
uses a known Z=0 plane, downward cameras at world centers (0,0,10) and (2,0,10),
independent calibrations, 24 fitting pairs and eight distinct held-out observations.
It first requires full algebraic/physical acceptance. It then changes only backward
maximum range to one world unit, requiring physical rejection while the complete
algebraic diagnostics remain unchanged. It performs no real DTED navigation.