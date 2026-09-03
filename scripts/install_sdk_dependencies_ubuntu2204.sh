#!/usr/bin/env bash
set -Eeuo pipefail

readonly INSTALL_PREFIX="/usr/local"
jobs=2
keep_work=0
source_root=""

usage() {
  printf '%s\n' \
    "Usage: $0 [--jobs N] [--source-root PATH] [--keep-work]" \
    "" \
    "Builds the robo_manip SDK's pinned C++ runtime dependencies from" \
    "their upstream repositories. This installer supports Ubuntu 22.04" \
    "with ROS 2 Humble on x86-64." \
    "" \
    "--source-root is an INPUT directory containing the existing alg_dep" \
    "Git repositories. Nothing is installed into that directory." \
    "Without --source-root, the pinned sources are downloaded." \
    "" \
    "Ubuntu/ROS provide Boost 1.74 and the ordinary ROS dependencies." \
    "The ABI-pinned algorithm libraries are installed as ordinary locally" \
    "built system libraries under /usr/local. No private environment script" \
    "or motion-server-specific dependency prefix is created." \
    "" \
    "Install prefix: ${INSTALL_PREFIX}" \
    "Default parallel jobs: ${jobs}"
}

while (($#)); do
  case "$1" in
    --jobs)
      [[ $# -ge 2 && "$2" =~ ^[1-9][0-9]*$ ]] || {
        printf '%s\n' '--jobs requires a positive integer' >&2
        exit 2
      }
      jobs=$2
      shift 2
      ;;
    --source-root)
      [[ $# -ge 2 ]] || { usage >&2; exit 2; }
      source_root=$2
      shift 2
      ;;
    --keep-work)
      keep_work=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      printf 'Unknown argument: %s\n' "$1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ ! -r /etc/os-release ]]; then
  printf '%s\n' 'Cannot determine the operating-system release.' >&2
  exit 1
fi
# shellcheck disable=SC1091
source /etc/os-release
if [[ "${ID:-}" != "ubuntu" || "${VERSION_ID:-}" != "22.04" ]]; then
  printf 'This installer supports Ubuntu 22.04 only (found %s %s).\n' \
    "${ID:-unknown}" "${VERSION_ID:-unknown}" >&2
  exit 1
fi
if [[ "$(uname -m)" != "x86_64" ]]; then
  printf 'This installer supports x86-64 only (found %s).\n' "$(uname -m)" >&2
  exit 1
fi
if [[ ! -r /opt/ros/humble/setup.bash ]]; then
  printf '%s\n' \
    'ROS 2 Humble is required at /opt/ros/humble.' \
    'Install ROS 2 Humble first, then rerun this script.' >&2
  exit 1
fi

run_as_root() {
  if ((EUID == 0)); then
    "$@"
  elif command -v sudo >/dev/null 2>&1; then
    sudo "$@"
  else
    printf 'Root permission is required to run: %q' "$1" >&2
    printf ' %q' "${@:2}" >&2
    printf '\n' >&2
    exit 2
  fi
}

readonly apt_packages=(
  build-essential
  ca-certificates
  cmake
  git
  libassimp-dev
  libboost-chrono-dev
  libboost-filesystem-dev
  libboost-serialization-dev
  libboost-thread-dev
  libconsole-bridge-dev
  libeigen3-dev
  liborocos-kdl-dev
  libyaml-cpp-dev
  pkg-config
  ros-humble-ament-cmake
  ros-humble-eigen3-cmake-module
  ros-humble-kdl-parser
  ros-humble-urdf
  ros-humble-urdfdom
  ros-humble-urdfdom-headers
)

missing_packages=()
for package in "${apt_packages[@]}"; do
  if ! dpkg-query -W -f='${db:Status-Status}' "${package}" 2>/dev/null |
      grep -qx installed; then
    missing_packages+=("${package}")
  fi
done

if ((${#missing_packages[@]})); then
  quoted_packages=$(printf ' %q' "${missing_packages[@]}")
  if ((EUID == 0)); then
    apt-get update
    apt-get install -y "${missing_packages[@]}"
  elif command -v sudo >/dev/null 2>&1; then
    sudo apt-get update
    sudo apt-get install -y "${missing_packages[@]}"
  else
    printf '%s\n' \
      'Required Ubuntu packages are missing and sudo is unavailable.' \
      'Run this command once, then rerun the installer:' \
      "  sudo apt-get update && sudo apt-get install -y${quoted_packages}" >&2
    exit 2
  fi
fi

# The delivered liblibplaco.so has a direct DT_NEEDED entry for the Ubuntu
# 22.04 Boost.Filesystem ABI. ROS 2 Humble uses the same Ubuntu Boost 1.74
# packages, so reuse that system library instead of building a private Boost
# that could place two Boost ABIs in one process.
if ! dpkg-query -W -f='${db:Status-Status}\n' \
    libboost-filesystem1.74.0 2>/dev/null | grep -qx installed; then
  printf '%s\n' \
    'Ubuntu Boost.Filesystem 1.74 runtime is unavailable.' \
    'Install libboost-filesystem1.74.0, then rerun this installer.' >&2
  exit 1
fi

if [[ -n "${source_root}" ]]; then
  source_root=$(realpath -e -- "${source_root}")
  readonly local_repositories=(
    eiquadprog
    hpp-fcl
    jrl-cmakemodules
    nlopt
    octomap
    pinocchio
    ruckig
    toppra
    trac_ik
  )
  for repository_name in "${local_repositories[@]}"; do
    if ! git -C "${source_root}/${repository_name}" rev-parse \
        --is-inside-work-tree >/dev/null 2>&1; then
      printf 'Required local Git repository is missing: %s\n' \
        "${source_root}/${repository_name}" >&2
      exit 1
    fi
  done
fi

printf '%s\n' \
  "Algorithm source input: ${source_root:-download pinned upstream sources}" \
  "Dependency install output: ${INSTALL_PREFIX}" \
  'Boost provider: Ubuntu 22.04 system Boost 1.74 (shared with ROS 2 Humble)'

# Authenticate once up front instead of prompting once per dependency.
if ((EUID != 0)); then
  run_as_root -v
fi

work_dir=$(mktemp -d -t robot-motion-dependencies.XXXXXXXX)
cleanup() {
  if ((keep_work)); then
    printf 'Keeping dependency source/build directory: %s\n' "${work_dir}"
  else
    rm -rf -- "${work_dir}"
  fi
}
trap cleanup EXIT

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u
# Keep the build deterministic without creating a persistent private runtime
# environment. /usr/local is CMake's standard source-install prefix.
export CMAKE_PREFIX_PATH="${INSTALL_PREFIX}:/opt/ros/humble"
export PKG_CONFIG_PATH="${INSTALL_PREFIX}/lib/pkgconfig:${INSTALL_PREFIX}/lib/x86_64-linux-gnu/pkgconfig:/opt/ros/humble/lib/pkgconfig:/opt/ros/humble/lib/x86_64-linux-gnu/pkgconfig"
export LD_LIBRARY_PATH="${INSTALL_PREFIX}/lib:${INSTALL_PREFIX}/lib/x86_64-linux-gnu:/opt/ros/humble/lib:/opt/ros/humble/lib/x86_64-linux-gnu"

clone_exact() {
  local name=$1
  local repository=$2
  local expected_commit=$3
  local destination="${work_dir}/src/${name}"
  local checkout_repository="${repository}"
  local attempt

  if [[ -n "${source_root}" ]]; then
    local local_name="${name}"
    if [[ "${name}" == "trac-ik" ]]; then
      local_name=trac_ik
    fi
    checkout_repository="${source_root}/${local_name}"
    if ! git -C "${checkout_repository}" cat-file -e \
        "${expected_commit}^{commit}"; then
      printf 'Pinned commit %s is not available in %s.\n' \
        "${expected_commit}" "${checkout_repository}" >&2
      return 1
    fi
  fi

  mkdir -p -- "${work_dir}/src"
  for attempt in 1 2 3; do
    rm -rf -- "${destination}"
    git init --quiet "${destination}"
    git -C "${destination}" remote add origin "${checkout_repository}"
    if git -C "${destination}" fetch --quiet --depth 1 origin "${expected_commit}" &&
        git -C "${destination}" checkout --quiet --detach FETCH_HEAD; then
      break
    fi
    if ((attempt == 3)); then
      printf 'Failed to fetch %s at commit %s after three attempts.\n' \
        "${repository}" "${expected_commit}" >&2
      return 1
    fi
    printf 'Retrying clone of %s (%d/3).\n' \
      "${name}" "$((attempt + 1))" >&2
  done

  local actual_commit
  actual_commit=$(git -C "${destination}" rev-parse HEAD)
  if [[ "${actual_commit}" != "${expected_commit}" ]]; then
    printf 'Pinned commit mismatch for %s: expected %s, got %s\n' \
      "${name}" "${expected_commit}" "${actual_commit}" >&2
    return 1
  fi

  if [[ -f "${destination}/.gitmodules" ]]; then
    if [[ -n "${source_root}" ]] &&
        git -C "${destination}" ls-tree HEAD cmake |
          grep -q '^160000 commit '; then
      git -C "${destination}" config submodule.cmake.url \
        "${source_root}/jrl-cmakemodules"
      git -C "${destination}" -c protocol.file.allow=always \
        submodule update --init --depth 1 cmake >&2
    elif [[ -z "${source_root}" ]]; then
      # clone_exact is normally called inside command substitution. Keep
      # human-readable submodule progress out of stdout so the caller receives
      # exactly one value: the source directory printed below.
      git -C "${destination}" submodule update --init --recursive --depth 1 >&2
    fi
  fi
  printf '%s\n' "${destination}"
}

configure_build_install() {
  local name=$1
  local source_dir=$2
  shift 2
  local build_dir="${work_dir}/build/${name}"
  cmake -S "${source_dir}" -B "${build_dir}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}" \
    -DCMAKE_INSTALL_LIBDIR=lib \
    -DCMAKE_PREFIX_PATH="${CMAKE_PREFIX_PATH}" \
    -DCMAKE_BUILD_RPATH="${INSTALL_PREFIX}/lib" \
    -DCMAKE_INSTALL_RPATH= \
    -DCMAKE_INSTALL_RPATH_USE_LINK_PATH=OFF \
    "$@"
  cmake --build "${build_dir}" --parallel "${jobs}"
  run_as_root cmake --install "${build_dir}"
  run_as_root ldconfig
}

install_nlopt() {
  local commit=9e44e525370646def8152e73bb5c53a6531f6f7e
  local source_dir
  source_dir=$(clone_exact nlopt https://github.com/stevengj/nlopt.git "${commit}")
  configure_build_install nlopt "${source_dir}" \
    -DBUILD_SHARED_LIBS=ON \
    -DNLOPT_CXX=ON \
    -DNLOPT_FORTRAN=OFF \
    -DNLOPT_GUILE=OFF \
    -DNLOPT_JAVA=OFF \
    -DNLOPT_MATLAB=OFF \
    -DNLOPT_OCTAVE=OFF \
    -DNLOPT_PYTHON=OFF \
    -DNLOPT_SWIG=OFF \
    -DNLOPT_TESTS=OFF
}

install_ruckig() {
  local commit=cb99a04ce488f83701aaee6efd9c9f0d36a3d43b
  local source_dir
  source_dir=$(clone_exact ruckig https://github.com/pantor/ruckig.git "${commit}")
  configure_build_install ruckig "${source_dir}" \
    -DBUILD_SHARED_LIBS=ON \
    -DBUILD_BENCHMARK=OFF \
    -DBUILD_CLOUD_CLIENT=OFF \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_PYTHON_MODULE=OFF \
    -DBUILD_TESTS=OFF
}

install_toppra() {
  local commit=cbbc89d46208fddfaa3e1aab52ab44553751b510
  local source_dir
  source_dir=$(clone_exact toppra https://github.com/hungpham2511/toppra.git "${commit}")
  configure_build_install toppra "${source_dir}/cpp" \
    -DBUILD_SHARED_LIBS=ON \
    -DBUILD_TESTING=OFF \
    -DBUILD_WITH_GLPK=OFF \
    -DBUILD_WITH_PINOCCHIO=OFF \
    -DBUILD_WITH_qpOASES=OFF \
    -DOPT_MSGPACK=OFF \
    -DPYTHON_BINDINGS=OFF
}

install_jrl_cmakemodules() {
  # Install this explicitly before eiquadprog/hpp-fcl/pinocchio. Otherwise
  # those projects may invoke CMake FetchContent during configuration and hang
  # on an implicit, unpinned GitHub download.
  local commit=52fb166d9500d6c7841a7ea96312e9bf8d000360
  local source_dir
  source_dir=$(clone_exact \
    jrl-cmakemodules \
    https://github.com/jrl-umi3218/jrl-cmakemodules.git \
    "${commit}")
  configure_build_install jrl-cmakemodules "${source_dir}"
}

install_eiquadprog() {
  local commit=ec402b4dbcce32fd936fd39a3c6fc32f08b35a54
  local source_dir
  source_dir=$(clone_exact eiquadprog https://github.com/stack-of-tasks/eiquadprog.git "${commit}")
  configure_build_install eiquadprog "${source_dir}" \
    -DBUILD_SHARED_LIBS=ON \
    -DBUILD_TESTING=OFF \
    -DTRACE_SOLVER=OFF
}

install_octomap() {
  # hpp-fcl 2.4.4 exports an OctoMap dependency even when optional integration
  # is disabled, so install the matching small system library first.
  local commit=d417c181868be79931ec94fd1a407c323e9f0fd3
  local source_dir
  source_dir=$(clone_exact octomap https://github.com/OctoMap/octomap.git "${commit}")
  configure_build_install octomap "${source_dir}" \
    -DBUILD_DYNAMICETD3D_SUBPROJECT=OFF \
    -DBUILD_OCTOVIS_SUBPROJECT=OFF
}

install_hpp_fcl() {
  local commit=1c6f0a1d9c8d47914ab2196845327b3836de4b32
  local source_dir
  source_dir=$(clone_exact hpp-fcl https://github.com/humanoid-path-planner/hpp-fcl.git "${commit}")
  configure_build_install hpp-fcl "${source_dir}" \
    -DBUILD_PYTHON_INTERFACE=OFF \
    -DBUILD_TESTING=OFF \
    -DHPP_FCL_HAS_QHULL=OFF \
    -DINSTALL_DOCUMENTATION=OFF
}

install_pinocchio() {
  local commit=ed3bb75ce96cf26e84aebd8b73785407950a0f1f
  local source_dir
  source_dir=$(clone_exact pinocchio https://github.com/stack-of-tasks/pinocchio.git "${commit}")
  configure_build_install pinocchio "${source_dir}" \
    -DBUILD_ADVANCED_TESTING=OFF \
    -DBUILD_BENCHMARK=OFF \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_PYTHON_INTERFACE=OFF \
    -DBUILD_TESTING=OFF \
    -DBUILD_UTILS=OFF \
    -DBUILD_WITH_AUTODIFF_SUPPORT=OFF \
    -DBUILD_WITH_CASADI_SUPPORT=OFF \
    -DBUILD_WITH_CODEGEN_SUPPORT=OFF \
    -DBUILD_WITH_COLLISION_SUPPORT=ON \
    -DBUILD_WITH_EXTRA_SUPPORT=OFF \
    -DBUILD_WITH_OPENMP_SUPPORT=OFF \
    -DBUILD_WITH_SDF_SUPPORT=OFF \
    -DBUILD_WITH_URDF_SUPPORT=ON \
    -DENABLE_TEMPLATE_INSTANTIATION=ON \
    -DINSTALL_DOCUMENTATION=OFF
}

install_trac_ik() {
  # aprotyas/trac_ik is the ROS 2 port whose package.xml identifies this ABI
  # as 0.1.0. It has no immutable 0.1.0 tag, so pin the exact commit.
  local commit=b7b432529a2f43a57dbcebec4b2d5923781668a7
  local source_dir
  source_dir=$(clone_exact trac-ik https://github.com/aprotyas/trac_ik.git "${commit}")
  if ! grep -q '<version>0.1.0</version>' \
      "${source_dir}/trac_ik_lib/package.xml"; then
    printf '%s\n' 'Pinned TRAC-IK source does not identify version 0.1.0.' >&2
    exit 1
  fi
  configure_build_install trac-ik "${source_dir}/trac_ik_lib" \
    -DBUILD_TESTING=OFF
}

install_nlopt
install_ruckig
install_toppra
install_jrl_cmakemodules
install_eiquadprog
install_octomap
install_hpp_fcl
install_pinocchio
install_trac_ik

required_paths=(
  lib/libeiquadprog.so
  lib/libhpp-fcl.so
  lib/libnlopt.so
  lib/liboctomap.so
  lib/libpinocchio_default.so.3.9.0
  lib/libpinocchio_parsers.so.3.9.0
  lib/libruckig.so
  lib/libtoppra.so
  lib/libtrac_ik_lib.so
  lib/cmake/eiquadprog/eiquadprogConfig.cmake
  lib/cmake/hpp-fcl/hpp-fclConfig.cmake
  lib/cmake/nlopt/NLoptConfig.cmake
  lib/cmake/pinocchio/pinocchioConfig.cmake
  lib/cmake/ruckig/ruckig-config.cmake
  lib/cmake/toppra/toppraConfig.cmake
  share/cmake/jrl-cmakemodules/jrl-cmakemodulesConfig.cmake
  share/trac_ik_lib/cmake/trac_ik_libConfig.cmake
)
for relative_path in "${required_paths[@]}"; do
  if [[ ! -e "${INSTALL_PREFIX}/${relative_path}" ]]; then
    printf 'Expected installed SDK dependency is missing: %s\n' \
      "${INSTALL_PREFIX}/${relative_path}" >&2
    exit 1
  fi
done

private_boost=$(find "${INSTALL_PREFIX}/lib" "${INSTALL_PREFIX}/lib/x86_64-linux-gnu" \
  -maxdepth 1 -name 'libboost_*.so*' -print -quit 2>/dev/null || true)
if [[ -n "${private_boost}" ]]; then
  printf '%s\n' \
    "Unexpected locally installed Boost library: ${private_boost}" \
    'Remove that local Boost copy; Boost must come from Ubuntu 22.04.' >&2
  exit 1
fi

printf '%s\n' \
  "Pinned SDK dependencies installed in ${INSTALL_PREFIX}." \
  'The input alg_dep directory was not modified.' \
  'No dependency-specific environment script is required.' \
  'CMake and the dynamic linker now discover the libraries normally.'
