"""Exercise the workflow inventory shell, not a simulated Windows build.

Only Alire/pacman responses are stubbed. Real hosted compilation, execution and
PE inspection remain separate mandatory qualification evidence.
"""
import os
from pathlib import Path
import subprocess
import tempfile
import textwrap
import unittest

ROOT = Path(__file__).resolve().parents[2]
WORKFLOW = ROOT / ".github/workflows/windows-post-merge.yml"
PREFIX = "mingw-w64-x86_64-"
REQUIRED = ("gcc", "cc-libs", "libgcc", "libstdc++", "libgfortran",
            "libquadmath", "libgomp", "opencv", "pkgconf")
DLLS = {"libgcc": "libgcc_s_seh-1.dll", "libstdc++": "libstdc++-6.dll",
        "libgfortran": "libgfortran-5.dll", "libquadmath": "libquadmath-0.dll",
        "libgomp": "libgomp-1.dll", "libatomic": "libatomic-1.dll"}


def inventory_shell():
    text = WORKFLOW.read_text()
    start = text.index("      - name: Verify Alire-deployed MSYS2 packages")
    stop = text.index("      - name: Build and run with isolated runtime", start)
    return textwrap.dedent(text[start:stop].split("        run: |\n", 1)[1])


class WindowsInventoryTests(unittest.TestCase):
    def run_inventory(self, missing="", fault="", atomic=False):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            stub = root / "alr"
            stub.write_text("""#!/usr/bin/env python3
import os
import sys
prefix = 'mingw-w64-x86_64-'
required = """ + repr(REQUIRED) + """
dlls = """ + repr(DLLS) + """
args = sys.argv[1:]
assert args[:3] == ['exec', '--', 'pacman'], args
args = args[3:]
with open(os.environ['CALLS'], 'a') as log:
    log.write(' '.join(args) + '\\n')
installed = {prefix + p for p in required} | {prefix + 'openblas'}
if os.environ['ATOMIC'] == '1':
    installed.add(prefix + 'libatomic')
installed.discard(prefix + os.environ['MISSING'])
fault = os.environ['FAULT']
if args == ['-Qq']:
    print('\\n'.join(sorted(installed)))
elif args[0] in ('-Q', '-Qi'):
    if any(p not in installed for p in args[1:]):
        sys.exit(1)
    print('\\n'.join(p + ' 16.2.0-6' for p in args[1:]))
elif args[0] == '-Ql':
    package = args[1]
    if package not in installed:
        sys.exit(1)
    if fault != 'file-list':
        print(package + ' /mingw64/bin/' + dlls[package[len(prefix):]])
elif args[0] == '-Qqo':
    if fault == 'unowned':
        sys.exit(1)
    owner = next(p for p, dll in dlls.items() if args[1] == '/mingw64/bin/' + dll)
    print(prefix + ('wrong-owner' if fault == 'wrong-owner' else owner))
elif args[0] == '-Qk':
    if fault == 'missing-file':
        sys.exit(1)
elif args == ['-Dk']:
    if fault == 'database':
        sys.exit(1)
else:
    raise AssertionError(args)
""")
            stub.chmod(0o755)
            calls = root / "calls"
            env = dict(os.environ, PATH=f"{root}:{os.environ['PATH']}",
                       CALLS=str(calls), MISSING=missing, FAULT=fault,
                       ATOMIC="1" if atomic else "0")
            result = subprocess.run(["bash", "-c", inventory_shell()], env=env,
                                    text=True, capture_output=True)
            return result, calls.read_text()

    def test_split_packages_pass_without_gcc_libs(self):
        result, calls = self.run_inventory()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertNotIn(PREFIX + "gcc-libs", calls)
        self.assertIn("-Dk\n", calls)
        for package in REQUIRED:
            self.assertIn(PREFIX + package, calls)
        for package, dll in DLLS.items():
            if package != "libatomic":
                self.assertIn(f"-Qqo /mingw64/bin/{dll}\n", calls)
                self.assertIn(f"-Qk {PREFIX}{package}\n", calls)

    def test_missing_required_package_fails_closed(self):
        for package in REQUIRED:
            with self.subTest(package=package):
                result, calls = self.run_inventory(missing=package)
                self.assertNotEqual(result.returncode, 0)
                self.assertNotIn("-Dk\n", calls)

    def test_missing_or_wrongly_owned_dll_fails_closed(self):
        for fault in ("file-list", "unowned", "wrong-owner", "missing-file"):
            with self.subTest(fault=fault):
                result, calls = self.run_inventory(fault=fault)
                self.assertNotEqual(result.returncode, 0)
                self.assertNotIn("-Dk\n", calls)

    def test_database_failure_is_not_ignored(self):
        result, calls = self.run_inventory(fault="database")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("-Dk\n", calls)

    def test_installed_optional_atomic_is_verified(self):
        result, calls = self.run_inventory(atomic=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("-Qqo /mingw64/bin/libatomic-1.dll\n", calls)

    def test_real_windows_build_and_linkage_gates_remain(self):
        text = WORKFLOW.read_text()
        for gate in ("toolchain --select gnat_native=16.1.0 gprbuild=26.0.1",
                     "run: sh scripts/test.sh", "test -f lib/libopencv_calib3d_shim.dll",
                     "test -f lib/libopencv_calib3d_shim.dll.a",
                     "test ! -f lib/libopencv_calib3d_shim.a",
                     "*gnat_native*) exit 1", "*mingw64*/bin/g++*)",
                     'test -f "$import_library"', 'objdump.exe" -p',
                     "DLL Name.*opencv_(calib3d|geometry)", "DLL Name.*opencv_core_shim"):
            self.assertIn(gate, text)
        self.assertNotIn("continue-on-error", text)
        self.assertNotIn("if: ", text)
        self.assertNotIn("pacman -S", inventory_shell())
        self.assertNotIn("|| true", inventory_shell())
        runner = (ROOT / "scripts/run_native.sh").read_text()
        self.assertIn('PATH="$root/lib:$core_lib:$mingw_bin:$PATH"', runner)


if __name__ == "__main__":
    unittest.main()