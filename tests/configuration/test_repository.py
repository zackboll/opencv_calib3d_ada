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


if __name__ == "__main__":
    unittest.main()
