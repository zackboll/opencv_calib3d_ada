# CALIB3D-CI-031: Windows MSYS2 runtime inventory correction

## Verified starting point and historical failure

Corrective branch `corrective/windows-msys2-runtime-inventory` was created directly
from fetched main `8021a405721ab07c5fb4b3daf1f1d62cfa81503a`, exact tree
`bb1991b393bc0512f02619c25e2658e76528a76a`. This is a separate corrective PR,
not a modification to CALIB3D-030 PR #31 or its feature branch.

The historical [Windows run 40437352808](https://github.com/zackboll/opencv_calib3d_ada/actions/runs/40437352808),
[job 121969102808](https://github.com/zackboll/opencv_calib3d_ada/actions/runs/40437352808/job/121969102808),
failed in `Verify Alire-deployed MSYS2 packages` before compilation. The exact
command in the Windows workflow was:

```sh
alr exec -- pacman -Q \
  mingw-w64-x86_64-gcc \
  mingw-w64-x86_64-gcc-libs \
  mingw-w64-x86_64-cc-libs \
  mingw-w64-x86_64-libgfortran \
  mingw-w64-x86_64-libquadmath \
  mingw-w64-x86_64-opencv \
  mingw-w64-x86_64-pkgconf
```

Original output, preserved without relabeling it as success:

```text
error: package 'mingw-w64-x86_64-gcc-libs' was not found
mingw-w64-x86_64-gcc 16.2.0-6
mingw-w64-x86_64-cc-libs 16.2.0-6
mingw-w64-x86_64-libgfortran 16.2.0-6
mingw-w64-x86_64-libquadmath 16.2.0-6
mingw-w64-x86_64-opencv 5.0.0-7
mingw-w64-x86_64-pkgconf 1~3.0.10-1
```

The query exits 1 because one requested installed-package name is absent. The
original installation log nevertheless records installation of `libgcc`,
`libstdc++`, `libgfortran`, `libquadmath` and `libgomp`. Missing `gcc-libs` therefore
does **not** establish a missing GCC runtime. Build/test and PE steps were skipped;
the original run supplies no successful Windows runtime evidence.

The full original `gh run view --log` output is retained for this session at
`/tmp/calib3d-ci-031/original-windows-40437352808.log` (1,346 lines, 172,247 bytes;
SHA256 `301b519e67ff924376b2cf9d31f660c06980da7601568953b4a9c41a7ba8ba91`).
The linked original hosted run remains the historical source; no rerun erases it.

## Current upstream package/runtime mapping

Inspected MSYS2 GCC **16.2.0-6** package metadata and the
[GCC PKGBUILD](https://github.com/msys2/MINGW-packages/blob/5c1ca99730b358ba0915fc8eba7a29e6e169c378/mingw-w64-gcc/PKGBUILD).
The inspected PKGBUILD SHA256 is
`1a123ea062e01beb611c4375999492b0f41ef2d92d03e38cb29e7c23e855ad45`.
All names below have prefix `mingw-w64-x86_64-`; all DLLs are under `/mingw64/bin/`.

| Package | Current role / owned runtime DLL |
|---|---|
| [gcc-libs](https://packages.msys2.org/package/mingw-w64-x86_64-gcc-libs) | Still available, empty compatibility/dependency package; depends on exact-version cc-libs, libatomic, libgomp, libquadmath. Not required merely for its name. |
| [cc-libs](https://packages.msys2.org/package/mingw-w64-x86_64-cc-libs) | Empty C++ dependency package; depends on exact-version libgcc and libstdc++. Retained as a required installed component. |
| [libgcc](https://packages.msys2.org/package/mingw-w64-x86_64-libgcc) | `libgcc_s_seh-1.dll`; required by gcc, cc-libs and other language runtimes. |
| [libstdc++](https://packages.msys2.org/package/mingw-w64-x86_64-libstdc%2B%2B) | `libstdc++-6.dll`; required by gcc and cc-libs. |
| [libgfortran](https://packages.msys2.org/package/mingw-w64-x86_64-libgfortran) | `libgfortran-5.dll`; provides fc-libs for OpenBLAS; depends on exact-version libgcc/libquadmath. |
| [libquadmath](https://packages.msys2.org/package/mingw-w64-x86_64-libquadmath) | `libquadmath-0.dll`; required by libgfortran. |
| [libgomp](https://packages.msys2.org/package/mingw-w64-x86_64-libgomp) | `libgomp-1.dll`; provides omp for OpenCV/OpenBLAS. Required explicitly. |
| [libatomic](https://packages.msys2.org/package/mingw-w64-x86_64-libatomic) | `libatomic-1.dll`; optional GCC dependency, not needed by the observed deployment. If installed, its files/ownership are verified too. |

[OpenCV 5.0.0-7](https://packages.msys2.org/package/mingw-w64-x86_64-opencv)
depends on omp and [OpenBLAS](https://packages.msys2.org/package/mingw-w64-x86_64-openblas);
OpenBLAS depends on cc-libs, fc-libs and omp. Upstream metadata is evidence of the
packaging contract, **not a substitute for querying the actual runner**. The
corrected hosted step logs the installed versions and `pacman -Qi` dependency
graph, `-Ql` file lists, `-Qqo` exact owners, `-Qk` file existence and mandatory
`-Dk` dependency/database consistency. `-Qqo` is the quiet ownership form of `-Qo`.
Git Bash argument conversion is disabled only for the literal MSYS ownership path,
so native Alire forwards `/mingw64/bin/...` to its own MSYS2 pacman unchanged.

## Fail-closed correction and regression protection

The inventory now requires gcc, cc-libs, libgcc, libstdc++, libgfortran,
libquadmath, libgomp, opencv and pkgconf. It never installs gcc-libs, upgrades
packages, tolerates missing required queries or ignores database errors. Each
required DLL must occur in its expected package's installed file list, be owned
by that exact package, and pass installed-file checking. All commands retain
`set -eu`. If a package later declares libatomic mandatory, missing it fails
`pacman -Dk`; if installed, its ownership/file checks run as well.

Six focused Python regression groups execute the actual inventory shell with
explicitly stubbed Alire/pacman responses: supported split inventory without
gcc-libs; each missing required package; missing file-list entry, unowned DLL,
wrong DLL owner and missing package file; database failure; installed optional
libatomic; and preservation of real build/link/runtime gates. These are shell
policy tests, **not fake successful Windows builds**. Baseline configuration tests
were **40/40**; corrected configuration tests are **46/46 PASS** locally.

The selected Alire GNAT toolchain, MSYS2 MinGW64 g++, real `scripts/test.sh`,
runtime-isolation launcher, external shim DLL/import library creation, static-shim
rejection, configured native import libraries, PE inspection and external
OpenCV/Core-shim linkage checks remain enabled and unchanged. Trigger policy is
unchanged: main pushes, plus PRs changing this workflow file, no manual dispatch.
Production Ada/C++, private ABI, dependencies, AUnit registrations, examples and
CALIB3D-030 changes are not part of this correction.

## Qualification and review gate

Exact corrective head/tree, PR URL and actual hosted Windows run/step results are
recorded in the corrective PR qualification ledger. Package-query success alone
is insufficient: Windows must execute the real build/tests and DLL/PE gates.
Local Linux shell regression tests cannot establish Windows runtime qualification.

Leave the corrective PR open, non-draft and unmerged with auto-merge disabled for
independent review. No post-merge success is claimed before an authorized merge.
After independent review and separate merge authorization, verify the exact merge
and main tree, require successful corrected-main Windows CI as superseding (not
rewritten historical) evidence, then reconcile PR #31 by a normal merge commit.
Do not rebase/force-push/replace PR #31 or reconcile it before this prerequisite.
The feature must then repeat affected qualification and pass exact-new-head CI;
its own Windows qualification remains post-merge-only.