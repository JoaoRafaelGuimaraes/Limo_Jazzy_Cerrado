#!/usr/bin/env bash
# Compila so o que a simulacao precisa: limo_description (oficial) + limo_cerrado_sim.
# Os outros pacotes do limo_ros2 (limo_base, limo_car) dependem de hardware/Gazebo Classic.
set -euo pipefail
cd "$(dirname "$(readlink -f "$0")")"
# o venv do mrs_ws no PATH faz o colcon/ament usar um python sem as dependencias do ROS
export PATH="$(echo "$PATH" | tr ':' '\n' | grep -v '/root/mrs_ws/.venv/bin' | paste -sd:)"; unset VIRTUAL_ENV || true
set +u; source /opt/ros/jazzy/setup.bash; set -u
# padrao de varredura real do MID-360 (25 MB, fora do git); sem ele o launch cai para livox_model:=grid
src/limo_cerrado_sim/scripts/fetch_livox_pattern.sh || echo "AVISO: nao foi possivel baixar o padrao do MID-360"
colcon build --symlink-install --packages-up-to limo_cerrado_sim "$@"
echo
echo "Pronto. Em cada terminal:  source /root/limo_ws/install/setup.bash"
