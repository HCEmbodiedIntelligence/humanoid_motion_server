#!/usr/bin/env bash
# Optional display dependencies only; never replace the system NumPy/Pinocchio.
set -euo pipefail
simulation_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
simulation_venv="${1:-${simulation_root}/.simulation-venv}"
python3 -m venv --without-pip --system-site-packages "${simulation_venv}"
"${simulation_venv}/bin/python" -m pip install -r "${simulation_root}/requirements-simulation.txt"
printf '\nActivate with: source %q/bin/activate\n' "${simulation_venv}"
printf 'Then source the ROS workspace and launch humanoid_motion_server simulation.launch.py.\n'
