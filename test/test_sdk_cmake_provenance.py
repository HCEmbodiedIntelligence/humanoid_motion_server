"""Reproduce a mixed-prefix RPATH cycle without changing ROS or system files."""
import argparse
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import textwrap
import unittest


parser = argparse.ArgumentParser()
parser.add_argument("--dependency-module", type=Path, required=True)
OPTIONS, TEST_ARGS = parser.parse_known_args() if __name__ == "__main__" else (None, [])


@unittest.skipIf(OPTIONS is None, "Run through CTest with the dependency module path")
class SdkCmakeProvenanceTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix="sdk-cmake-provenance-")
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.ros_prefix = self.root / "conflicting-ros-prefix"
        library_dir = self.ros_prefix / "lib"
        self.ros_config = library_dir / "cmake/pinocchio"
        self.ros_config.mkdir(parents=True)
        # CMake's cycle detector needs real files with the same SONAMEs in
        # both prefixes. The fixture is never installed or used for motion.
        for name in ("libruckig.so", "libpinocchio_parsers.so.3.9.0",
                     "libpinocchio_default.so.3.9.0"):
            shutil.copyfile(Path("/usr/local/lib") / name, library_dir / name)
        (self.ros_config / "pinocchioConfig.cmake").write_text(textwrap.dedent(f'''\
            foreach(component pinocchio_default pinocchio_parsers)
              add_library(pinocchio::${{component}} SHARED IMPORTED)
              set_target_properties(pinocchio::${{component}} PROPERTIES
                IMPORTED_LOCATION "{library_dir}/lib${{component}}.so.3.9.0")
            endforeach()
            set(pinocchio_FOUND TRUE)
            '''))
        (self.ros_config / "pinocchioConfigVersion.cmake").write_text(
            'set(PACKAGE_VERSION "3.9.0")\n'
            'set(PACKAGE_VERSION_EXACT TRUE)\nset(PACKAGE_VERSION_COMPATIBLE TRUE)\n')
        self.project = self.root / "project"
        self.project.mkdir()
        (self.project / "main.cpp").write_text("int main() { return 0; }\n")
        (self.project / "CMakeLists.txt").write_text(textwrap.dedent('''\
            cmake_minimum_required(VERSION 3.16)
            project(sdk_dependency_provenance LANGUAGES CXX)
            if(PRELOAD_CONFLICT)
              find_package(pinocchio 3.9.0 EXACT REQUIRED CONFIG)
            endif()
            if(USE_PINNED_POLICY)
              include("${SDK_DEPENDENCY_MODULE}")
            else()
              find_package(ruckig 0.17.3 EXACT REQUIRED CONFIG)
              find_package(pinocchio 3.9.0 EXACT REQUIRED CONFIG)
            endif()
            add_executable(probe main.cpp)
            target_link_libraries(probe ruckig::ruckig pinocchio::pinocchio_parsers)
            get_target_property(selected_pinocchio pinocchio::pinocchio_parsers IMPORTED_LOCATION_RELEASE)
            file(WRITE "${CMAKE_BINARY_DIR}/selected-provider.txt" "${selected_pinocchio}")
            '''))

    def configure(self, folder, *, fixed=True, preload=False):
        environment = dict(os.environ)
        environment["CMAKE_PREFIX_PATH"] = str(self.ros_prefix) + ":" + environment.get("CMAKE_PREFIX_PATH", "")
        return subprocess.run([
            "cmake", "-S", str(self.project), "-B", str(folder),
            f"-DUSE_PINNED_POLICY={'ON' if fixed else 'OFF'}",
            f"-DPRELOAD_CONFLICT={'ON' if preload else 'OFF'}",
            # Preserve the install path of a symlink-installed CMake file:
            # its CMAKE_CURRENT_LIST_DIR determines the installed SDK prefix.
            f"-DSDK_DEPENDENCY_MODULE={OPTIONS.dependency_module.absolute()}",
            # Model both a ROS-first environment and an old valid cache entry.
            f"-Dpinocchio_DIR={self.ros_config}"],
            env=environment, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)

    def test_legacy_lookup_reproduces_runtime_search_path_cycle(self):
        result = self.configure(self.root / "legacy-build", fixed=False)
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertIn("cycle in the", result.stdout)
        self.assertIn("libruckig.so", result.stdout)
        self.assertIn("libpinocchio_parsers.so.3.9.0", result.stdout)

    def test_pinned_lookup_repairs_cache_and_removes_the_cycle(self):
        build = self.root / "fixed-build"
        result = self.configure(build)
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertNotIn("Cannot generate a safe runtime search path", result.stdout)
        self.assertNotIn("eigenpy", result.stdout)
        self.assertTrue((build / "selected-provider.txt").read_text().startswith("/usr/local/lib/"))
        cache = (build / "CMakeCache.txt").read_text()
        self.assertIn("pinocchio_DIR:PATH=/usr/local/lib/cmake/pinocchio", cache)
        self.assertIn("hpp-fcl_DIR:PATH=/usr/local/lib/cmake/hpp-fcl", cache)
        self.assertIn("octomap_DIR:PATH=/usr/local/share/octomap", cache)

    def test_preloaded_conflicting_target_is_rejected(self):
        result = self.configure(self.root / "preloaded-build", preload=True)
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertIn("Conflicting SDK target pinocchio::", result.stdout)


if __name__ == "__main__":
    unittest.main(argv=[__file__, *TEST_ARGS])
