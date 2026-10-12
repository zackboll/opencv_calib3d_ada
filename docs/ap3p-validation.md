# AP3P local qualification

OpenCV 4.10.0 / calib3d, local Linux. This is not a claim that remote CI or
the pinned matrix has passed.

## Windows baseline

Task 011 merged at `3611fe81dfe755b304f4e175ffced93acd42e4ee`. Main-push
cross-platform run `38157062615` passed. Windows post-merge run
`38157062743` failed in `pacman -Sy --needed` before compilation:

```text
installing mingw-w64-x86_64-libgcc (16.2.0-5)
breaks dependency mingw-w64-x86_64-libgcc=16.2.0-4
required by cc-libs, libgfortran and libquadmath
```

The same conflict applies to `libstdc++`. No Task 011 binding was compiled.
The repository does not mix those runtime versions. Windows remains
unqualified until a consistent MSYS2 transaction succeeds. One rerun is
deferred until the repository state is consistent; this is not a polling loop.

## Measured local results

99 AUnit registrations, 99 executed, 99 passed, 0 failed assertions,
0 unexpected errors. New cases: known pose, five-coefficient distortion,
four retained hypotheses, world-to-camera frame, fourth-point residual,
count/degeneracy rejection, native zero result, and C/Ada layout.

Known nonidentity pose, zero distortion, two hypotheses:

```text
angle 0
translation 3.68e-15
camera center 3.39e-15
three-point pixels 3.22e-13
fourth-point pixels 2.84e-13
```

Five-coefficient distortion, two hypotheses:

```text
angle 7.92e-7 rad
translation 6.11e-6
camera center 6.29e-6
three-point pixels 2.68e-4
fourth-point pixels 3.06e-4
```

The front-parallel fixture retains 4 hypotheses. The known-pose alternate
candidate has a fourth-point residual of about 247 pixels and is retained.
The valid zero-solution fixture returns `1 .. 0`. Repeated, collinear,
identical-bearing, exactly coplanar, and overflowing coordinates raise
`OpenCV_Error`.

A radial `k1=0.2` fixture whose ideal rays are exactly coplanar has distorted
normalized coordinates that are not collinear. Native `undistortPoints` on
OpenCV 4.10 recovers them with maximum error `1.44021e-5`. Pixel-only
preflight would treat those pixels as safe bearings; the corrected preflight
inspects the recovered rays and still accepts this nonzero residual. The
raw solver returns a bounded hypothesis set. The portable exact-coplanar
rejection uses normalized `y=0`, whose bearing product is bitwise zero.
The ideal `y=0.5` line is mathematically collinear, but its unit-bearing
product is not portable across libm and is not the rejection oracle.

Private ABI: 46 declarations and imports. Compiler layout: size 200,
alignment 8, count at 0, candidates at 8, pose stride 48, candidate offsets
8, 56, 104, 152. All six fields of all four candidates interchange.

Raw production and fault paths pass, including 20 fault scenarios at
checkpoints 50..53 and post-native controls for zero, count above four,
list-size mismatch, bad vector shape, and nonfinite output. Ada translates
the same 20 faults. Actual-shim production and fault-injection ASan+UBSan
both passed with leak detection and halt-on-error. Linked OpenCV and Core
are not claimed to be fully instrumented.
