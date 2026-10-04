# PnP source review bootstrap

This file records the review targets; Task 001 must verify exact source hashes and
reachable behavior before qualification claims are made.

| OpenCV | Peeled revision | Native area |
|---|---|---|
| 4.1.0 | `371bba8f54560b374fbcd47e7e02f015ac4969ad` | `modules/calib3d` |
| 4.10.0 | `71d3237a093b60a27601c20e9ee6c3e52154e8b1` | `modules/calib3d` |
| 5.0.0 | `40738fb16ceddb5fb3fea747585f7ce6abb0605b` | `modules/geometry` |

Review at minimum:

- public `solvePnPRansac` declaration and fixed method behavior;
- implementation input `checkVector`/conversion paths;
- EPNP path and the exactly-four-point RANSAC special case;
- false-return behavior and inlier representation;
- `projectPoints` input/output shape and Float64 preservation;
- `Rodrigues` behavior used by camera-center conversion;
- OpenCV 4/5 module/header/library split.

`solvePnPRefineLM` is deliberately deferred from the common baseline because it is
not in the OpenCV 4.1 public API.
