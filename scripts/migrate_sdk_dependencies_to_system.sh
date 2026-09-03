#!/usr/bin/env bash
set -Eeuo pipefail

readonly OLD_PREFIX="/opt/local/humanoid_motion_server/sdk-deps"
readonly SYSTEM_PREFIX="/usr/local"
script_dir=$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
jobs=2
source_root=""
archive_old=0
verify_only=0

usage() {
  printf '%s\n' \
    "Usage: $0 [--source-root PATH] [--jobs N] [--verify-only] [--archive-old]" \
    "" \
    "Rebuilds the pinned motion SDK dependencies into the standard" \
    "/usr/local system layout, verifies them without the legacy private" \
    "prefix, and optionally archives the old prefix after verification." \
    "" \
    "Without --source-root, the installer downloads pinned upstream sources." \
    "--verify-only skips rebuilding and only verifies the system installation." \
    "--archive-old renames the managed old prefix instead of deleting it."
}

while (($#)); do
  case "$1" in
    --source-root)
      [[ $# -ge 2 ]] || { usage >&2; exit 2; }
      source_root=$2
      shift 2
      ;;
    --jobs)
      [[ $# -ge 2 && "$2" =~ ^[1-9][0-9]*$ ]] || {
        printf '%s\n' '--jobs requires a positive integer' >&2
        exit 2
      }
      jobs=$2
      shift 2
      ;;
    --archive-old)
      archive_old=1
      shift
      ;;
    --verify-only)
      verify_only=1
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

if ((!verify_only)); then
  installer_arguments=(--jobs "${jobs}")
  if [[ -n "${source_root}" ]]; then
    installer_arguments+=(--source-root "${source_root}")
  fi
  "${script_dir}/install_sdk_dependencies_ubuntu2204.sh" \
    "${installer_arguments[@]}"
fi

# Verify in a shell state that cannot accidentally discover the legacy prefix.
unset HUMANOID_MOTION_SDK_DEPS_PREFIX
for variable_name in CMAKE_PREFIX_PATH LD_LIBRARY_PATH PKG_CONFIG_PATH; do
  current_value=${!variable_name:-}
  filtered_value=""
  IFS=: read -r -a path_entries <<<"${current_value}"
  for path_entry in "${path_entries[@]}"; do
    [[ -z "${path_entry}" || "${path_entry}" == "${OLD_PREFIX}"/* || \
       "${path_entry}" == "${OLD_PREFIX}" ]] && continue
    filtered_value+="${filtered_value:+:}${path_entry}"
  done
  printf -v "${variable_name}" '%s' "${filtered_value}"
  export "${variable_name}"
done

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u

"${script_dir}/check_sdk_runtime" \
  --sdk-root "${script_dir}/../vendor/robo_manip"

probe_dir=$(mktemp -d -t robot-motion-system-probe.XXXXXXXX)
cleanup_probe() {
  rm -rf -- "${probe_dir}"
}
trap cleanup_probe EXIT
cat >"${probe_dir}/CMakeLists.txt" <<'CMAKE'
cmake_minimum_required(VERSION 3.16)
project(robot_motion_system_dependency_probe LANGUAGES CXX)
find_package(ruckig 0.17.3 EXACT REQUIRED CONFIG)
find_package(toppra 0.6.8 EXACT REQUIRED CONFIG)
find_package(NLopt 2.10.1 EXACT REQUIRED CONFIG)
find_package(trac_ik_lib 0.1.0 EXACT REQUIRED CONFIG)
find_package(eiquadprog 1.3.2 EXACT REQUIRED CONFIG)
find_package(hpp-fcl 2.4.4 EXACT REQUIRED CONFIG)
find_package(pinocchio 3.9.0 EXACT REQUIRED CONFIG)
CMAKE
cmake -S "${probe_dir}" -B "${probe_dir}/build"

if ((archive_old)) && [[ -d "${OLD_PREFIX}" ]]; then
  old_marker="${OLD_PREFIX}/share/humanoid_motion_server-sdk-deps/managed-prefix"
  if [[ ! -f "${old_marker}" ]]; then
    printf '%s\n' \
      "Refusing to archive an unrecognized directory: ${OLD_PREFIX}" \
      "Expected the old installer's marker: ${old_marker}" >&2
    exit 1
  fi
  archive_path="${OLD_PREFIX}.legacy.$(date +%Y%m%d%H%M%S)"
  if ((EUID == 0)); then
    mv -- "${OLD_PREFIX}" "${archive_path}"
  else
    sudo mv -- "${OLD_PREFIX}" "${archive_path}"
  fi
  printf 'Legacy dependency prefix archived at %s\n' "${archive_path}"
elif [[ -d "${OLD_PREFIX}" ]]; then
  printf '%s\n' \
    "Standard installation verified. Legacy prefix remains at ${OLD_PREFIX}." \
    "Remove any shell/systemd references to its setup.bash, then rerun this" \
    "migration with --verify-only --archive-old to move it aside safely."
fi

printf '%s\n' \
  "Migration verified: headers, libraries, and CMake packages use ${SYSTEM_PREFIX}." \
  'No motion-specific dependency environment needs to be sourced.'
