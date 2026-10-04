"""Execute configure against isolated metadata; no Alire generated state touched."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]


class ConfigureTests(unittest.TestCase):
    def configure(self, version, backend="calib3d", missing=None):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            scripts = root / "scripts"
            scripts.mkdir()
            shutil.copy(ROOT / "scripts/configure_opencv.sh", scripts)
            (scripts / "build_opencv_shim.sh").write_text("#!/bin/sh\nexit 0\n")
            include = root / "include"
            lib = root / "lib"
            lib.mkdir()
            header = "opencv2/geometry/3d.hpp" if backend == "geometry" else "opencv2/calib3d.hpp"
            (include / header).parent.mkdir(parents=True)
            if missing != "header":
                (include / header).touch()
            if missing != "library":
                (lib / f"libopencv_{backend}.so").touch()
            core = root / "core"
            (core / "cpp").mkdir(parents=True)
            (core / "cpp/opencv_core_module_bridge.hpp").touch()
            bin_dir = root / "bin"
            bin_dir.mkdir()
            (bin_dir / "uname").write_text("#!/bin/sh\nprintf 'Linux\\n'\n")
            metadata = bin_dir / "pkg-config"
            metadata.write_text(f"""#!/bin/sh
case "$1" in
  --exists) [ "$2" = opencv4 ] ;;
  --modversion) printf '%s\\n' '{version}' ;;
  --variable=includedir) printf '%s\\n' '{include}' ;;
  --variable=libdir) printf '%s\\n' '{lib}' ;;
  *) exit 1 ;;
esac
""")
            for executable in bin_dir.iterdir():
                executable.chmod(0o755)
            env = dict(os.environ, PATH=f"{bin_dir}:{os.environ['PATH']}",
                       PKG_CONFIG=str(metadata), OPENCV_CORE_ALIRE_PREFIX=str(core))
            result = subprocess.run(["sh", str(scripts / "configure_opencv.sh")],
                                    env=env, text=True, capture_output=True)
            generated = root / "config/opencv_calib3d_install.gpr"
            return result, generated.read_text() if generated.exists() else ""

    def test_reviewed_backend_mapping(self):
        for version, backend in [("4.1.0", "calib3d"), ("4.10.0", "calib3d"), ("5.0.0", "geometry")]:
            with self.subTest(version=version):
                result, generated = self.configure(version, backend)
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertIn(f'Native_Backend := "{backend}";', generated)
                self.assertIn(f'-lopencv_{backend}', generated)

    def test_unreviewed_versions_fail_closed(self):
        for version in ["4.0.0", "3.4.0", "5.1.0", "6.0.0", "4.10junk.0", "4.10", "garbage"]:
            with self.subTest(version=version):
                result, generated = self.configure(version)
                self.assertNotEqual(result.returncode, 0)
                self.assertEqual(generated, "")

    def test_missing_header_fails_closed(self):
        result, generated = self.configure("4.10.0", missing="header")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("Missing native header", result.stderr)
        self.assertEqual(generated, "")

    def test_missing_backend_library_fails_closed(self):
        result, generated = self.configure("5.0.0", "geometry", missing="library")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("Missing native backend library", result.stderr)
        self.assertEqual(generated, "")