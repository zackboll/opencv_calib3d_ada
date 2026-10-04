#!/usr/bin/env python3
"""Static repository consistency checks; this is not an Ada/native build."""
from pathlib import Path
import os
import re
import sys
import tomllib

from workflow_topology import check_workflows

ROOT = Path(__file__).resolve().parent.parent


def check(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def bridge_not_vendored() -> None:
    for directory, children, files in os.walk(ROOT):
        children[:] = [n for n in children if n not in {".git", "alire", "obj"}]
        check("opencv_core_module_bridge.hpp" not in files, "do not vendor Core's bridge")


def main() -> None:
    manifests = [tomllib.loads((ROOT / p).read_text()) for p in
                 ("alire.toml", "tests/alire.toml", "examples/alire.toml")]
    production = manifests[0]
    dependencies = {k for group in production["depends-on"] for k in group}
    check({"opencv_core", "opencv", "pkg_config"} <= dependencies, "missing production dependency")
    check(not dependencies & {"opencv_features", "opencv_imgproc", "opencv_geometry", "aunit"},
          "production dependency boundary changed")
    commits = [m["pins"][-1]["opencv_core"]["commit"] for m in manifests]
    check(len(set(commits)) == 1 and re.fullmatch(r"[0-9a-f]{40}", commits[0]) is not None,
          "Core pins must be identical full commits")

    header = (ROOT / "cpp/opencv_calib3d_shim.h").read_text()
    ada = (ROOT / "src/internal/opencv-calib3d-internal-c_api.ads").read_text()
    cpp = (ROOT / "cpp/opencv_calib3d_shim.cpp").read_text()
    declared = set(re.findall(r"\b(opencv_calib3d_\w+)\s*\(", header))
    imported = set(re.findall(r'External_Name\s*=>\s*"(opencv_calib3d_\w+)"', ada))
    # Metadata/error functions are imported too, so exact sets should match.
    check(declared == imported, f"C/Ada import mismatch: {declared ^ imported}")
    check(all(re.search(r"\b" + re.escape(name) + r"\s*\(", cpp) for name in declared),
          "missing C++ export")

    bridge_not_vendored()
    check_workflows(ROOT / ".github/workflows")
    check(not list((ROOT / "src").rglob("opencv.ads")), "do not redeclare Core's root package")
    check(not list((ROOT / "src").rglob("opencv-geometry*.ad*")), "do not collide with OpenCV.Geometry")
    for path in list((ROOT / "src").rglob("*.ads")) + list((ROOT / "src").rglob("*.adb")):
        check("External_Name" not in path.read_text() or "/internal/" in path.as_posix(),
              f"C import leaked into public Ada: {path}")

    tests = (ROOT / "tests/src/calib3d_tests.adb").read_text()
    registrations = re.findall(r"Result\.Add_Test\s*\(Caller\.Create", tests)
    check(len(registrations) == 10, "update documented AUnit inventory when changing tests")

    configure = (ROOT / "scripts/configure_opencv.sh").read_text()
    check("backend=calib3d" in configure and "backend=geometry" in configure,
          "OpenCV 4/5 backend split is missing")
    check("opencv2/calib3d.hpp" in configure and "opencv2/geometry/3d.hpp" in configure,
          "backend headers are not checked")

    print(f"PASS: manifests, Core pin {commits[0][:12]}, {len(declared)} ABI declarations/imports, "
          f"10 AUnit registrations, Core ownership, 4/5 backend split, CI topology")


if __name__ == "__main__":
    try:
        main()
    except (OSError, KeyError, ValueError) as error:
        print(f"FAIL: {error}", file=sys.stderr)
        sys.exit(1)
