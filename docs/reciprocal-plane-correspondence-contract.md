# Reciprocal plane-constrained correspondence verification (Task 026)

```text
Camera A pixel --> Plane --> P_A --> Camera B reprojection
       |                     |
       |                Compare XYZ
       |                     |
Camera B pixel --> Plane --> P_B --> Camera A reprojection
                             |
                     Reciprocal acceptance
```

## Supplied geometry and mandatory independent policy

`Verify_Reciprocal_Plane_Correspondences` independently calls the public
`Verify_Plane_Constrained_Correspondences` exactly once in each direction for a
nonempty request. A->B uses A pixels, calibration, Brown model and pose as source,
B as target, and `Options.Forward`. B->A reverses **all** camera arguments and uses
`Options.Backward`. Ada reserves the word `reverse`; the public record therefore
uses `Backward`, while the diagnostic accessor is `Reverse_Verification`.

Both policies are the existing `Plane_Correspondence_Verification_Options`, not
copies of that type. Incidence, range, represented-plane residual and target pixel
error limits remain independent. All [Task 025 semantics](plane-correspondence-contract.md)
and [Task 024 source semantics](world-plane-intersection-contract.md) apply.

`Maximum_World_Separation` is mandatory, finite, nonnegative and in caller world
length units. Comparison is inclusive `separation <= maximum`, without defaults,
hidden tolerances, epsilon adjustment or minimum stereo baseline. Same-center
cameras with different rotations are valid when their forward rays reach the
supplied plane. No pose, calibration, plane or correspondence is estimated.

The frame convention is always `X_camera = R * X_world + t`, with Rodrigues
rotation and `C_world = -R^T * t`. Translation is **not** camera location. Both
poses and the Cartesian plane must use the same frame and length units.

## Complete validation and atomic batches

Before either verifier, validate matched lengths, INT32 count bound, every pixel,
both calibrations (finite positive focal lengths), both five-coefficient Brown
models, all twelve pose components, finite plane anchor/nonzero finite normal,
both complete intersection policies, both target pixel limits, and separation
limit. A shared private policy validator preserves Task 025 validation conditions,
order and error text. Malformed empty requests still raise `OpenCV_Error`.
Valid empty input returns owned bounds `1..0` with no native calls.

Each nonempty directional call uses one source pixel batch and at most one native
positive-depth target projection batch. Target projection is not inside the
per-correspondence loop. Native undistortion, Rodrigues, camera-center or projection
errors propagate as `OpenCV_Error`: the function returns no partial array, and a
failed assignment preserves the previously assigned complete result.

The implementation adds no native operation, C++ code, C ABI layout, production
test hook or dependency. The private ABI remains 48 operations. Core owns all
temporary Mats inside reused APIs; native borrowing remains callback scoped.

## Four distinct acceptance/availability stages

1. **Source-plane acceptance**: each ray independently passes its own Task 024
   forward intersection policy. Rejected sources cannot supply coordinates.
2. **Directional target acceptance**: its source point also has representable
   positive target depth and a projected distorted-pixel residual within that
   direction's threshold. Each complete Task 025 result is retained unchanged.
3. **Both-source separation availability**: if both source intersections accepted,
   evaluate the distance between their independently reconstructed positions,
   even if either directional target check failed. No rejected source is read.
4. **Final reciprocal acceptance**: both Task 025 results accepted, both sources
   accepted, separation available, and separation within the caller's limit.

Small world disagreement never upgrades a rejected directional result.

## Separation arithmetic and representability

Obtain `P_A` and `P_B` only through their guarded source accessors. Check each
Float64 subtraction `P_A(axis)-P_B(axis)` and reuse `Scaled_Vector_Norm` (scale
before squaring; verify the final multiplication). Never square original large
differences. Exact zero is valid, tiny positive separation is retained, and no
invented epsilon clamps it to zero. Unrepresentable subtraction or norm makes
only that correspondence's separation unavailable; it never publishes NaN,
infinity, a saturation or fabricated zero.

The exception translation block contains **only** checked subtraction and the
existing scaled norm plus private result assignment. All native calls and all
public guarded reads are outside it. No native/accessor error is reclassified as
separation unavailability. Existing Float64 normalization, intermediate rounding
and component underflow limitations remain; this is not extended precision.
Large-origin rounding can erase a genuine small geometric displacement. A zero
represented separation does not prove physical coordinate accuracy.

## Owned results, guards and deterministic precedence

Output bounds are `1..N`. Positional pairing is independent of input lower bounds,
including multi-element arrays ending at `Positive'Last`. `Original_Position` is
the original one-based correspondence position, not an Ada input subscript.
Owned copies and slices retain it.

`Directions_Evaluated`, `Both_Source_Intersections_Accepted`,
`World_Separation_Evaluated`, `Disposition`, `Original_Position` and `Accepted`
are always available. A default result has no directions or separation and is
not accepted. `Forward_Verification` and `Reverse_Verification` require evaluated
directions and return complete owned Task 025 values, including rejected source
diagnostics and all available target depth/residual diagnostics. Their original
guards remain authoritative. `World_Separation` requires its availability flag.
Unavailable guarded accessors raise `OpenCV_Error`, not a storage-zero diagnostic.

The first applicable disposition wins:

1. Both sources rejected -> `Both_Sources_Rejected`.
2. Only forward source rejected -> `Forward_Source_Rejected`.
3. Only reverse source rejected -> `Reverse_Source_Rejected`.
4. Both directional results rejected -> `Both_Directions_Rejected`.
5. Only forward directional result rejected -> `Forward_Verification_Rejected`.
6. Only reverse directional result rejected -> `Reverse_Verification_Rejected`.
7. Separation unavailable -> `World_Separation_Numerically_Unavailable`.
8. Separation exceeds the maximum -> `World_Separation_Rejected`.
9. Otherwise -> `Reciprocal_Plane_Accepted`.

Thus a target rejection may summarize a result with unavailable separation; check
the independent availability predicate and full directional records for details.
No source/target diagnostics are overwritten by the summary.

## Coordinate provenance and accepted-landmark adapter

`First_Source_World_Position` and `Second_Source_World_Position` require their
respective source acceptance, not final acceptance. They remain independently
derived from the two camera rays; neither point is moved, fitted or averaged.
`Accepted_First_Source_World_Position` and `Accepted_Second_Source_World_Position`
additionally require final reciprocal acceptance. The first-camera value is a
deterministic downstream representative, **not** a fused, statistically optimal or
preferred physical truth estimate. No midpoint or covariance is created.

`Reciprocal_Plane_Landmark_Observations` returns the existing `Accepted_Landmark_Set`
for corresponding later-image pixels. It validates counts and **every** later
pixel, including rejected tracks. Only final reciprocal-accepted records contribute
their accepted first-camera representative, the later observation, and stored
original position. Slicing/rebasing is supported via positional pairing and stored
identities. No acceptance gives empty one-based world/pixel/identity arrays. No
later pose is solved. Planar landmarks alone do not guarantee robust six-DOF
relocalization or meaningful depth diversity.

## What reciprocal agreement does not establish

A plane agreement check does not establish absolute ground truth, correct
correspondences, physically accurate poses or statistical independence. Both
independent rays can agree on the same wrong plane if the supplied model is wrong.
Systematic calibration, pose, plane and matching errors can be shared. Near-grazing
intersection is ill-conditioned; iterative Brown inversion is approximate.
Qualified synthetic fixture tolerances are not universal accuracy bounds.

This task adds no homography estimator, image warp, DTED parser, DEM lookup,
Earth model, geodesy, bundle adjustment, SLAM, navigation fusion, optimizer,
feature matcher or statistical uncertainty estimate. All example observations
are synthetic and make no physical terrain-registration/navigation-accuracy claim.