#!/bin/bash
# Robotu surer. Calisan bir simulasyona baglanir; ortami kendisi hazirlar.
#
#   ./tools/sur.sh [sure_s] [ly]
#
#   ly: 1.0 -> 0.40 m/s, 0.75 -> 0.30 m/s, 0.5 -> 0.20 m/s
HERE=$(cd "$(dirname "$0")" && pwd)
source /opt/ros/jazzy/setup.bash
source $HERE/../ws/install/setup.bash
exec python3 $HERE/drive_go2.py "$@"
