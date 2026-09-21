#!/usr/bin/env bash
# Compila so o que a simulacao precisa: limo_description (oficial) + limo_cerrado_sim.
# Os outros pacotes do limo_ros2 (limo_base, limo_car) dependem de hardware/Gazebo Classic.
set -euo pipefail
cd "$(dirname "$(readlink -f "$0")")"
# o venv do mrs_ws no PATH faz o colcon/ament usar um python sem as dependencias do ROS
export PATH="$(echo "$PATH" | tr ':' '\n' | grep -v '/root/mrs_ws/.venv/bin' | paste -sd:)"; unset VIRTUAL_ENV || true
set +u; source /opt/ros/jazzy/setup.bash; set -u
colcon build --symlink-install --packages-up-to limo_cerrado_sim "$@"
echo
echo "Pronto. Em cada terminal:  source /root/limo_ws/install/setup.bash"
