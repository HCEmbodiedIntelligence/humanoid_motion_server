#!/usr/bin/env bash
set -euo pipefail

# ROS 2 Humble on Ubuntu 22.04 deliberately requires an opt-in for cppcheck
# 2.7.  The server's production source set is small and completes quickly.
export AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1
exec /opt/ros/humble/bin/ament_cppcheck "$@"
