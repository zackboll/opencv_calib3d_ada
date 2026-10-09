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
    check(commits[0] == "4da9d35ea21e1b2efe96296243ea668b488c6326", "qualified Core pin changed")
    check(production["version"] == "0.1.0-dev", "review task must not bump version")

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
    check(len(registrations) == 80, "update documented AUnit inventory when changing tests")
    check(len(declared) == 43, "update private ABI inventory when changing exports")
    layout = (ROOT / "tests/layout/calib3d_layout.c").read_text()
    check("calib3d_test_triangulation_layout" in layout and
          "calib3d_test_fill_triangulation" in tests and
          all(f"offsetof(opencv_calib3d_triangulated_point,{field})" in layout
              for field in ("status", "x", "y", "z", "depth_first", "depth_second",
                            "error_first", "error_second")), "triangulation layout helper coverage")
    check("cv::triangulatePoints(p1,p2,points1,points2,h)" in cpp and
          "h.type()!=CV_64FC1" in cpp, "triangulation Float64 path/output schema")
    check("count >= 15" in cpp and "cv::findFundamentalMat(first, second, cv::FM_RANSAC" in cpp,
          "fundamental must prevent the hidden 8..14 LMeDS fallback")
    check("final_fundamental_inlier(value->matrix" in cpp and "value->inliers.size() >= 7" in cpp,
          "fundamental requires final-F Float64 classification and seven final inliers")
    fundamental = (ROOT / "cpp/fundamental_profile.hpp").read_text()
    check("threshold * threshold" in fundamental and "static_cast<float>(squared)" in fundamental,
          "native comparison squares before converting to Float32")
    check("confidence < DBL_EPSILON" in fundamental and "confidence > 1 - DBL_EPSILON" in fundamental,
          "fundamental confidence must prevent native substitution")
    helpers = {p.name for p in (ROOT / "tests/cpp").glob("*test.*")}
    check(helpers == {"header_test.c", "profile_test.cpp", "homography_profile_test.cpp",
                      "fundamental_profile_test.cpp", "essential_profile_test.cpp",
                      "triangulation_profile_test.cpp"}, "update helper inventory")
    check("count >= 6" in cpp and "cv::findEssentialMat(first,second,identity,cv::RANSAC" in cpp,
          "Essential requires six for true subset RANSAC and identity intrinsics")
    check(re.search(r"#if CV_VERSION_MAJOR >= 5\s+//[^#]+"
                    r"options->normalized_epipolar_threshold,1000,native_mask\);\s+#else\s+"
                    r"cv::Mat e = cv::findEssentialMat\([^#]+"
                    r"options->normalized_epipolar_threshold,native_mask\);\s+#endif", cpp),
          "Essential uses legacy 4.x overload and required fixed-1000 5.0 overload")
    check("final_essential_inlier(value->matrix" in cpp and "value->inliers.size() >= 5" in cpp,
          "Essential requires final-E Float64 Sampson classification and five support")
    check("std::numeric_limits<double>::max(),mask,cv::noArray()" in cpp,
          "relative pose must avoid hidden 50-unit depth cutoff")
    check("final_homography_inlier(value->matrix" in cpp and "native_mask.copyTo" not in cpp,
          "homography must independently classify final H")
    check("count >= 5" in cpp and "cv::findHomography(source, destination, cv::RANSAC" in cpp,
          "homography robust-only minimum/method changed")
    check({"opencv_calib3d_undistort_normalized", "opencv_calib3d_rotation_matrix_of"} <= declared,
          "missing camera geometry ABI")
    check("20, 1.0e-12" in cpp and "cv::TermCriteria::COUNT | cv::TermCriteria::EPS" in cpp,
          "fixed undistortion convergence policy changed")
    check("cv::fisheye::" not in cpp, "camera rays use only the standard model")
    check("opencv_calib3d_refine_pose_iterative" in declared,
          "missing iterative refinement ABI")
    check("true, cv::SOLVEPNP_ITERATIVE" in cpp, "refinement must use the initial guess")
    check("solvePnPRefineLM(" not in cpp and "solvePnPRefineVVS(" not in cpp,
          "refinement must retain the common solvePnP path")

    configure = (ROOT / "scripts/configure_opencv.sh").read_text()
    check("backend=calib3d" in configure and "backend=geometry" in configure,
          "OpenCV 4/5 backend split is missing")
    check("opencv2/calib3d.hpp" in configure and "opencv2/geometry/3d.hpp" in configure,
          "backend headers are not checked")

    print(f"PASS: manifests, Core pin {commits[0][:12]}, {len(declared)} ABI declarations/imports, "
          f"{len(registrations)} AUnit registrations, Core ownership, 4/5 backend split, CI topology")


if __name__ == "__main__":
    try:
        main()
    except (OSError, KeyError, ValueError) as error:
        print(f"FAIL: {error}", file=sys.stderr)
        sys.exit(1)
