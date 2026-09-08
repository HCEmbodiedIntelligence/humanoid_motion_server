"""Exercise real ELF resolution with an incompatible same-SONAME Ruckig fixture.

All fixtures live in a temporary directory. No ROS/system library is modified,
and ldd only inspects dependencies; this test never starts a robot node.
"""
import argparse
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import textwrap
import unittest


parser = argparse.ArgumentParser()
parser.add_argument("--sdk-root", type=Path, required=True)
parser.add_argument("--checker", type=Path, required=True)
parser.add_argument("--node", type=Path, required=True)
parser.add_argument("--runtime", type=Path, required=True)
parser.add_argument("--installed-node", type=Path)
parser.add_argument("--installed-runtime", type=Path)
OPTIONS, TEST_ARGS = parser.parse_known_args() if __name__ == "__main__" else (None, [])
SYSTEM_PATHS = ["/usr/local/lib", "/usr/local/lib/x86_64-linux-gnu"]
ROS_PATHS = ["/opt/ros/humble/lib", "/opt/ros/humble/lib/x86_64-linux-gnu"]


@unittest.skipIf(OPTIONS is None, "Run through CTest or supply the SDK/binary command-line paths")
class SdkLoaderPolicyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temporary = tempfile.TemporaryDirectory(prefix="sdk-loader-policy-")
        cls.addClassCleanup(cls.temporary.cleanup)
        cls.root = Path(cls.temporary.name)
        cls.collision = cls.root / "incompatible-ros-library"
        cls.collision.mkdir()
        cls.real_ldd = shutil.which("ldd")
        subprocess.run([
            "cc", "-shared", "-fPIC", "-Wl,-soname,libruckig.so", "-x", "c", "-",
            "-o", str(cls.collision / "libruckig.so")],
            input="int incompatible_ruckig_fixture(void) { return 0; }\n",
            text=True, check=True, capture_output=True)
        cls.wrapper_dir = cls.root / "bin"
        cls.wrapper_dir.mkdir()
        wrapper = cls.wrapper_dir / "ldd"
        # Substitute only the checker child's ROS search directory, so the
        # regression is reproducible without writing anything into /opt/ros.
        wrapper.write_text(textwrap.dedent('''\
            #!/usr/bin/python3
            import os
            import sys
            failure = os.environ.get("SDK_TEST_FAILURE")
            if failure:
                if failure == "exit":
                    sys.exit(9)
                print(failure)
                sys.exit(0)
            paths = os.environ["LD_LIBRARY_PATH"].split(":")
            paths = [os.environ["SDK_TEST_COLLISION"] if p ==
                     "/opt/ros/humble/lib/x86_64-linux-gnu" else p for p in paths]
            os.environ["LD_LIBRARY_PATH"] = ":".join(paths)
            os.execv(os.environ["SDK_TEST_REAL_LDD"], ["ldd", *sys.argv[1:]])
            '''))
        wrapper.chmod(0o755)

    def environment(self):
        return dict(os.environ,
                    LD_LIBRARY_PATH=":".join([str(self.collision), *ROS_PATHS,
                                               os.environ.get("LD_LIBRARY_PATH", "")]).rstrip(":"))

    def ldd(self, path, environment=None):
        return subprocess.run([self.real_ldd, "-r", str(path)],
                              env=environment or self.environment(), text=True,
                              stdout=subprocess.PIPE, stderr=subprocess.STDOUT)

    def check_script(self, failure=None):
        environment = self.environment()
        environment.update(PATH=str(self.wrapper_dir) + os.pathsep + environment["PATH"],
                           SDK_TEST_REAL_LDD=self.real_ldd,
                           SDK_TEST_COLLISION=str(self.collision))
        if failure:
            environment["SDK_TEST_FAILURE"] = failure
        return subprocess.run(["bash", str(OPTIONS.checker), "--sdk-root",
                               str(OPTIONS.sdk_root), "--ldd-only"],
                              env=environment, text=True, stdout=subprocess.PIPE,
                              stderr=subprocess.STDOUT)

    def assert_clean_resolution(self, result):
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertNotRegex(result.stdout, r"not found|undefined symbol:")
        self.assertNotIn(str(self.collision), result.stdout)
        if "libruckig.so =>" in result.stdout:
            self.assertRegex(result.stdout,
                             r"libruckig\.so => /usr/local/lib/(?:x86_64-linux-gnu/)?libruckig\.so")

    def test_fixture_reproduces_the_reported_ruckig_symbol_failure(self):
        environment = self.environment()
        environment["LD_LIBRARY_PATH"] = ":".join([
            str(OPTIONS.sdk_root / "lib"), str(self.collision), *SYSTEM_PATHS, *ROS_PATHS])
        result = self.ldd(OPTIONS.sdk_root / "lib/libmotion_control.so", environment)
        self.assertIn(str(self.collision / "libruckig.so"), result.stdout)
        self.assertRegex(result.stdout, r"undefined symbol: .*ruckig")

    def test_checker_prefers_pinned_system_libraries_over_ros_and_inherited_paths(self):
        result = self.check_script()
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertIn("SDK verification passed.", result.stdout)
        self.assertEqual(result.stdout.count("Running ldd -r on "), 3)
        self.assertIn("SDK library search path: " + ":".join([
            str(OPTIONS.sdk_root.resolve() / "lib"), *SYSTEM_PATHS, *ROS_PATHS]), result.stdout)
        # The announced search path includes the test fixture; inspect only
        # actual ldd results when checking which providers were loaded.
        result.stdout = result.stdout.split("\n", 1)[1]
        self.assert_clean_resolution(result)

    def test_checker_still_rejects_real_errors(self):
        for failure in ("undefined symbol: test_missing_function",
                        "libmissing.so => not found", "exit",
                        "libboost_a.so.1.74.0 => /test/a\nlibboost_b.so.1.83.0 => /test/b"):
            with self.subTest(failure=failure):
                result = self.check_script(failure)
                self.assertNotEqual(result.returncode, 0, result.stdout)
                self.assertIn("SDK runtime verification failed.", result.stdout)

    def test_motion_binaries_keep_pinned_libraries_ahead_of_ros(self):
        paths = [OPTIONS.node, OPTIONS.runtime]
        paths.extend(path for path in (OPTIONS.installed_node, OPTIONS.installed_runtime) if path)
        for path in paths:
            with self.subTest(binary=str(path)):
                self.assertTrue(path.is_file(), str(path))
                dynamic = subprocess.check_output(["readelf", "-d", str(path)], text=True)
                match = re.search(r"\(RPATH\).*\[(.*?)\]", dynamic)
                self.assertIsNotNone(match, dynamic)
                search = match.group(1).split(":")
                for system_path in SYSTEM_PATHS:
                    self.assertIn(system_path, search)
                    for ros_path in ROS_PATHS:
                        if ros_path in search:
                            self.assertLess(search.index(system_path), search.index(ros_path))
                self.assert_clean_resolution(self.ldd(path))


if __name__ == "__main__":
    unittest.main(argv=[__file__, *TEST_ARGS])
