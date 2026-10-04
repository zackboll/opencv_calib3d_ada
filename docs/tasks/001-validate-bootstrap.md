# Task 001 — Validate Calib3D bootstrap and establish CI

## Goal

Take the generated bootstrap to a native green baseline without expanding scope.
Open one normal PR and stop at review.

Do not add new public APIs beyond the existing bootstrap pose slice. Do not merge,
tag, release, version-bump, amend, or force-push.

## Starting gate

Fetch the remote and record actual `origin/main`, worktree status and existing open
PRs. Create a feature branch such as:

```text
feature/001-pnp-bootstrap
```

The bootstrap ZIP itself has no native qualification claim.

## First build

Run serially:

```sh
python3 scripts/check_repository.py
python3 -m unittest discover -s tests/configuration -v
sh scripts/run_profile_tests.sh
for script in scripts/*.sh; do sh -n "$script"; done

alr -n build
alr -n -C tests build
alr -n -C tests exec -- sh ../scripts/run_native.sh bin/run_tests
alr test
alr -n exec -- sh scripts/run_sanitizers.sh
alr -n -C examples build
alr -n -C examples exec -- sh ../scripts/run_native.sh bin/pnp_synthetic
```

Do not run separate Alire builds concurrently against shared generated/config
storage.

Correct every actual compiler/link/runtime failure based on evidence. Do not remove
warnings-as-errors, tests, validation, ownership boundaries, or backend checks to
make the build green.

## Required architecture

Preserve:

```text
OpenCV 4.x: opencv2/calib3d.hpp      + opencv_calib3d
OpenCV 5.0: opencv2/geometry/3d.hpp  + opencv_geometry
Public Ada: OpenCV.Calib3D on both
```

Core owns application `Mat`. Native headers may be borrowed only through
`OpenCV.Core.Module_Interop` callbacks. Do not vendor Core's bridge.

## API contract review

Verify that native behavior and public docs agree on:

```text
X_camera = R * X_world + t
C_world  = -R^T * t
```

Confirm that `t` is never called camera position.

Review source for official OpenCV 4.1.0, 4.10.0 and 5.0.0 revisions and update
`docs/source-provenance.json` with immutable URLs/blob or SHA-256 hashes.
Document the exactly-four-point `solvePnPRansac` behavior instead of assuming the
EPNP flag means every internal stage uses EPNP.

## Tests

The bootstrap currently registers 10 AUnit cases. Correct that inventory if tests
are added. Preserve deterministic projection/camera-center oracles and robust
synthetic outlier coverage. Avoid exact pose/inlier-count assertions that are not
portable across OpenCV versions.

Raw boundary tests must use real Core handles. Test output initialization, invalid
schemas/options, result access and null destruction. No fake opaque pointers or
double destruction.

Compiler-derived C/Ada layout/interchange must cover intrinsics, distortion5 and
pose records.

## Sanitizers

Linux sanitizer qualification must instrument the actual Calib3D shim in production
and fault-injection variants with ASan+UBSan, leak detection enabled and no
suppressions. Test controls must not appear in production binaries.

## CI

PR CI must contain exactly:

```text
repository-checks
linux
macos
linux-sanitizers
```

Windows remains in `windows-post-merge.yml`, push-to-main only. Do not run Windows
on the PR.

After local and ordinary PR CI are green, dispatch the manual pinned matrix once on
a stable head:

```text
4.1.0 / calib3d
4.10.0 / calib3d
5.0.0 / geometry
```

Retain `WITH_ADE=OFF` for source builds.

## Git/PR gate

Use normal commits and pushes only. Suggested title:

```text
Bootstrap Calib3D robust pose binding
```

PR body must record actual test counts, native versions/backends, source revisions,
projection/camera-center/PnP evidence, ABI layout, sanitizer results, ordinary CI,
pinned matrix, and remaining limitations.

Stop when the PR is open, non-draft, unmerged, auto-merge disabled, with a clean
worktree and local/remote/PR heads equal. Do not begin pose refinement, homography,
DTED/geospatial integration, or Features integration in this task.
