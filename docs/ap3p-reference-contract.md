# AP3P independent-reference diagnostics

`Evaluate_AP3P_References` compares already obtained AP3P hypotheses with
additional 3-D/2-D observations. It does not solve AP3P, select a pose,
sort candidates, refine a pose, or infer a geographic position.

## Division of work

Native `Project_Points` performs perspective projection and the five
Brown/OpenCV distortion coefficients. Ada does not reimplement that
polynomial.

Ada obtains `R` once per hypothesis from `Rotation_Matrix_Of` and computes

```text
X_camera = R * X_world + t
```

`t` is the pose translation, not the camera center. A reference is eligible
for projection only when camera `Z` is finite and strictly greater than
zero. `Z = 0` and `Z < 0` are `Non_Positive_Depth` and are not sent to
native projection. There is no depth epsilon.

## Result matrix

The result is one-based:

```text
1 .. Hypotheses'Length  by  1 .. Reference_World'Length
```

Pairing follows position, not the source array bounds. Order is preserved.
`0` hypotheses or `0` references are valid after input validation and do
not call `projectPoints`. At most four hypotheses are accepted.

| Disposition | Camera depth | Pixel error |
|---|---|---|
| `Projected_Positive_Depth` | finite and `> 0` | finite and `>= 0` pixels |
| `Non_Positive_Depth` | finite and `<= 0` | undefined, stored as `0` |
| `Numerically_Unavailable` | undefined, stored as `0` | undefined, stored as `0` |

A stored zero on a nonprojected disposition is not a perfect projection.
Public results contain no NaN or infinity.

## Errors

`Numerically_Unavailable` covers specified Ada-side Float64 failures:
an unrepresentable camera-frame product or sum, or an unrepresentable
residual after a successful native projection. One such pair does not
invalidate another.

A failure inside `Project_Points`, including a native OpenCV exception,
invalid native output, or a nonfinite projected coordinate, propagates as
`OpenCV_Error`. The evaluation does not return a partial matrix and does
not translate that failure into `Numerically_Unavailable`. Isolating every
native failure per reference would require a separate checked projection
API; this task does not add one.

Malformed inputs, including a manually constructed hypothesis, raise
`OpenCV_Error` before evaluation. `Summarize_AP3P_References` likewise
rejects a caller-built record that violates the disposition table.

## Summary

`Summarize_AP3P_References` returns one one-based summary per hypothesis
row. Only projected references are passed to `Summarize_Reprojection`.
`Entire_Set_Evaluable` is true only when the row has at least one reference
and every reference was projected. An empty row has zero RMS and maximum
and is not a zero-error success. RMS over a subset is not an all-reference
score.

Positive depth and a small pixel error do not prove that a hypothesis is
the physical camera pose. Translation uses the units of the supplied world
coordinates.
