from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))
from workflow_topology import check_topology  # noqa: E402


class RepositoryTests(unittest.TestCase):
    def test_backend_split(self):
        text = (ROOT / "scripts/configure_opencv.sh").read_text()
        self.assertIn("backend=calib3d", text)
        self.assertIn("backend=geometry", text)
        self.assertIn("opencv2/calib3d.hpp", text)
        self.assertIn("opencv2/geometry/3d.hpp", text)

    def test_public_namespace_does_not_claim_geometry(self):
        self.assertTrue((ROOT / "src/opencv-calib3d.ads").exists())
        self.assertFalse(any((ROOT / "src").glob("opencv-geometry*.ads")))

    def test_bridge_not_vendored(self):
        hits = [p for p in ROOT.rglob("opencv_core_module_bridge.hpp") if "alire" not in p.parts]
        self.assertEqual(hits, [])

    def test_pnp_method_is_fixed(self):
        text = (ROOT / "cpp/opencv_calib3d_shim.cpp").read_text()
        self.assertIn("cv::SOLVEPNP_EPNP", text)
        self.assertNotIn("SOLVEPNP_DLS", text)
        self.assertNotIn("SOLVEPNP_UPNP", text)

    def test_pose_convention_documented(self):
        text = (ROOT / "docs/pnp-contract.md").read_text()
        self.assertIn("X_camera = R * X_world + t", text)
        self.assertIn("C_world = -R^T * t", text)

    def test_windows_has_no_pr_or_manual_trigger(self):
        text = (ROOT / ".github/workflows/windows-post-merge.yml").read_text()
        self.assertNotIn("pull_request:", text)
        self.assertNotIn("workflow_dispatch:", text)

    def test_compatibility_is_manual(self):
        text = (ROOT / ".github/workflows/opencv-compatibility.yml").read_text()
        self.assertIn("workflow_dispatch:", text)
        self.assertNotIn("pull_request:", text)

    def test_topology_parser_accepts_repository(self):
        check_topology(
            (ROOT / ".github/workflows/cross-platform.yml").read_text(),
            (ROOT / ".github/workflows/windows-post-merge.yml").read_text(),
            (ROOT / ".github/workflows/opencv-compatibility.yml").read_text(),
        )

    def test_no_features_dependency(self):
        text = (ROOT / "alire.toml").read_text()
        self.assertNotIn("opencv_features", text)

    def test_no_generated_binding_source(self):
        self.assertFalse(any(p.name.startswith("generate") for p in (ROOT / "scripts").iterdir()))

    def test_camera_rays_fixed_standard_policy(self):
        text = (ROOT / "cpp/opencv_calib3d_shim.cpp").read_text()
        self.assertIn("cv::TermCriteria::COUNT | cv::TermCriteria::EPS", text)
        self.assertIn("20, 1.0e-12", text)
        self.assertIn("images.isContinuous() ? images : images.clone()", text)
        self.assertNotIn("cv::fisheye::", text)

    def test_camera_ray_units_and_frames_documented(self):
        text = (ROOT / "docs/camera-rays-contract.md").read_text()
        for equation in ("X_world = R^T * (X_camera - t)", "d_world = R^T * d_camera",
                         "X_world(s) = C_world + s * d_world"):
            self.assertIn(equation, text)
        self.assertIn("dimensionless", text)

    def test_homography_final_classifier_is_independent_of_native_mask(self):
        text = (ROOT / "cpp/opencv_calib3d_shim.cpp").read_text()
        start = text.index("opencv_calib3d_status opencv_calib3d_find_homography_ransac(")
        end = text.index("opencv_calib3d_status opencv_calib3d_homography_result_found(", start)
        estimator = text[start:end]
        self.assertEqual(estimator.count("cv::findHomography("), 1)
        self.assertIn("cv::findHomography(source, destination, cv::RANSAC", estimator)
        self.assertIn("count >= 5", estimator)
        self.assertIn("original_source.at<cv::Vec2d>(i,0)", estimator)
        self.assertIn("original_destination.at<cv::Vec2d>(i,0)", estimator)
        self.assertIn("final_homography_inlier(value->matrix", estimator)
        self.assertIn("value->inliers.size() >= 4", estimator)
        self.assertNotRegex(estimator, r"native_mask\s*\.\s*(at|ptr|copyTo)")
        helper = (ROOT / "cpp/homography_profile.hpp").read_text()
        self.assertIn("std::hypot(xp - dx, yp - dy)", helper)
        self.assertIn("error <= threshold", helper)

    def test_homography_contract_records_precision_minimum_and_mask_difference(self):
        contract = (ROOT / "docs/homography-contract.md").read_text()
        for required in ("n=4 findHomography path bypasses RANSAC", "Float32 internally",
                         "original caller Float64", "destination-image", "h22 = 1",
                         "not a general 3-D", "4.1/4.10", "5.0", "upstream mask"):
            self.assertIn(required, contract)

    def test_topology_rejects_extra_windows_branch(self):
        cross = (ROOT / ".github/workflows/cross-platform.yml").read_text()
        windows = (ROOT / ".github/workflows/windows-post-merge.yml").read_text()
        compatibility = (ROOT / ".github/workflows/opencv-compatibility.yml").read_text()
        with self.assertRaises(ValueError):
            check_topology(cross, windows.replace("- main", "- main\n      - feature/**"), compatibility)

    def test_topology_rejects_extra_pr_job(self):
        cross = (ROOT / ".github/workflows/cross-platform.yml").read_text()
        windows = (ROOT / ".github/workflows/windows-post-merge.yml").read_text()
        compatibility = (ROOT / ".github/workflows/opencv-compatibility.yml").read_text()
        with self.assertRaises(ValueError):
            check_topology(cross + "\n  windows:\n    runs-on: windows-latest\n", windows, compatibility)


if __name__ == "__main__":
    unittest.main()
