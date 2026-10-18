# Calibrated Cartesian world-plane intersection (Task 024)

```text
Distorted image pixels
       |
Independent camera calibration
       |
Existing World_Bearing_Rays
       |
Caller-defined Cartesian plane
       |
Forward ray/plane intersection
       |
Incidence, range, residual gates
       |
Accepted world points
```

## Frames, supplied geometry, and API

```text
X_camera = R * X_world + t
C_world  = -R^T * t
X(s)     = C + s * D_unit
N_unit . (X - Anchor) = 0
s = dot(N_unit, Anchor - C) / dot(N_unit, D_unit)
```

Translation is t, not camera position. Plane coordinates, ray origins, accepted
coordinates, signed distances, forward ranges and plane residuals all use the
caller's Cartesian length units (meters only if the caller uses meters).
Directions and incidence cosines are dimensionless.

`World_Plane` contains `Anchor : Object_Point` and `Normal : World_Direction`.
`Intersect_World_Rays_With_Plane (Rays, Plane, Options)` is pure Ada geometry.
Arbitrary finite nonzero normals and ray directions use the existing scaled
`Unit_Vector`: divide by maximum absolute component before squaring, then divide
by the bounded scaled norm. No overflowing original norm is formed. Thus s is
Euclidean distance even when a caller constructs nonunit directions. Negating a
normal preserves the plane and distance; positive finite rescaling preserves
geometry to Float64 rounding, provided the supplied vector remains finite nonzero.

`Unproject_Pixels_Onto_World_Plane (Pixels, Intrinsics, Distortion, Pose, Plane,
Options)` calls existing `World_Bearing_Rays` and then exactly the direct
intersection primitive. It does not repeat Brown inversion, Rodrigues, or camera
center arithmetic. Camera calibration is supplied, not estimated here.

Both return owned `World_Plane_Intersection_Array` values, bounds `1..N` or `1..0`.
Pairing follows input position, not input subscripts; indexing subtracts before
adding (`First + (I - 1)`) including arrays ending at `Positive'Last`. There are
no handles, pointers, references into inputs, or finalization obligations.

## Validation and empty requests

All plane components and ray components must be finite. Normals and directions
must be nonzero. Validate the entire ray batch and normalize its directions before
evaluating any individual intersection. Invalid inputs raise `OpenCV_Error`, never
a per-ray disposition. Plane and policy validation also apply to empty arrays.

The pixel operation additionally validates pixels, calibration (finite positive
focal lengths), all five Brown coefficients and all six pose components through
the existing ray pipeline, even when empty. Valid empty calls bypass native
undistortion, Rodrigues and camera-center evaluation. Native errors propagate
atomically as `OpenCV_Error`; an unsuccessful function assignment cannot replace
any previously assigned results. No new native operation, C++ code, dependency,
solver, projection, or private ABI operation is introduced; ABI remains 48.

## Mandatory policy (no defaults)

| Field | Validation | Inclusive acceptance |
|---|---|---|
| `Minimum_Absolute_Incidence_Cosine` | finite, 0..1 | abs(dot(N_unit,D_unit)) >= limit |
| `Minimum_Forward_Range` | finite, >=0 | s >= limit, AND s > 0 |
| `Maximum_Forward_Range` | finite, >0, >= minimum | s <= limit |
| `Maximum_Absolute_Plane_Residual` | finite, >=0 | abs(dot(N_unit,X_represented-Anchor)) <= limit |

Zero minimum cosine disables grazing screening, not exact parallel rejection.
No comparison clamps a value, adjusts a threshold, adds an epsilon, or invents a
forward-distance epsilon. Incidence is the absolute **computed** normalized dot
product, not an angle in radians. Residual is measured on the actual represented
point, not an ideal point before reconstruction. A zero residual limit demands
an exactly zero computed residual and may reject ordinary rounding.

## Deterministic disposition and diagnostic precedence

Each stage must finish with finite representable arithmetic before its classifier
can apply. Arithmetic unavailability at any stage yields `Numerically_Unavailable`
and retains only diagnostics completed in earlier stages:

1. Compute denominator and publish absolute incidence availability.
2. Compute Anchor-C component differences and numerator. If denominator is exactly
   zero, numerator exactly zero yields `Coplanar_Ray`; otherwise `Parallel_Ray`.
   Never divide in either case. An unrepresentable numerator is numerical failure,
   not fabricated evidence of coplanarity/parallelism.
3. Divide a nonzero denominator; publish signed distance only after successful
   division. Exact zero gives `Intersection_At_Origin`, negative gives
   `Intersection_Behind_Ray`. These precede angular screening.
4. Positive s: incidence below limit gives `Grazing_Incidence_Rejected`.
5. Distance below minimum gives `Below_Minimum_Forward_Range`; distance above
   maximum gives `Above_Maximum_Forward_Range`.
6. Only now reconstruct C+s*D_unit and evaluate its represented residual.
   Residual above limit gives `Plane_Residual_Rejected`.
7. Otherwise publish position and `Forward_Intersection_Accepted`.

`Disposition`, `Incidence_Evaluated`, `Has_Signed_Ray_Distance`,
`Plane_Residual_Evaluated`, and `Accepted` are always available. The respective
numeric accessors `Absolute_Incidence_Cosine`, `Signed_Ray_Distance`, and
`Absolute_Plane_Residual` raise `OpenCV_Error` unless their predicate is true.
Only `Accepted=True` permits `Accepted_World_Position`. Private storage zeros
are never public evidence of geometry. A default result is numerically unavailable
with every availability predicate false. Negative distance is useful diagnostic
information, not forward acceptance. Rejected residuals remain available, but
their points are not published.

## Float64 and modeling limitations

Arithmetic helpers check every difference, product, accumulation, division and
reconstruction sum for nonfinite results, translating arithmetic-only
`Constraint_Error` into a dedicated local exception. Only that exception is
handled per ray; validation and native exceptions are outside its handler.
Nonzero products that underflow to zero, and nonzero numerators whose quotient
underflows to zero, are unavailable rather than fabricated zeros. Intermediate
overflow is rejected even when symbolic cancellation might produce a finite
answer. There is no saturation, extended-precision solver, or compensated dot
product. Normalization retains the existing Float64 scaled-vector contract.

Because normalized direction components have magnitude <=1, finite distance
scaling cannot overflow in isolation. It is nevertheless checked; origin-plus-
displacement, dot accumulation, offset subtraction and division can overflow.
Large finite origins can erase small displacements through rounding. A nonzero
represented plane residual can catch some such cases, not certify all coordinate
accuracy. Use suitable local Cartesian coordinates and explicit application policy.
Near-parallel geometry is ill-conditioned. Iterative undistortion is approximate;
the moderate all-five Brown fixture's 1e-8 world-unit recovery tolerance is a
qualified fixture tolerance, not a universal inverse-distortion accuracy bound.

This does not define geodetic latitude/longitude, ECEF, ENU/NED, DTED coordinates,
terrain elevation, or an Earth model. A real DTED/DEM surface is generally not one
infinite plane. Future applications may supply local planar patches, but this
primitive provides no triangle containment, terrain-tile selection, ray marching,
ray-mesh intersection, or DEM lookup. Knowing a camera pose and intersecting its
sightline with a supplied plane does not independently establish that the supplied
pose or plane is physically accurate. No plane estimation or fitting is performed.