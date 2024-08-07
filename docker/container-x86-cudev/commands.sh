#!/usr/bin/env bash

# Project-specific shell functions and commands.
#
# Roberto Masocco <r.masocco@dotxautomation.com>
#
# June 13, 2024

# Copyright 2024 dotX Automation s.r.l.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Add yours, some convenient ones are provided below.
# You can also source other files from sub-units included by this project.

# shellcheck disable=SC1090

# Source DUA commands
source ~/.dua_submod.sh
source ~/.dua_subtree.sh

# Routine to convert an angle in degrees [-180° +180°] to radians [-PI +PI].
function degrad {
  local angle_in_degrees="$1"
  angle_in_radians=$(python3 -c "import sys, math; angle=float(sys.argv[1]); print(math.radians((angle + 180) % 360 - 180))" "$angle_in_degrees")
  echo "$angle_in_radians"
}

# Routine to update dua-utils.
function utils-update {
  CURR_SHELL=$(ps -p $$ | awk 'NR==2 {print $4}')

  pushd || return
  cd /opt/ros/dua-utils || return
  git pull
  git submodule update --init --recursive
  rm -rf install
  colcon build --merge-install
  rm -rf build log
  source "install/local_setup.$CURR_SHELL"
  popd || return
}

# Routine to configure Cyclone DDS to use specific network interfaces.
function cyclonedds-configure {
  local cyclonedds_uri='<CycloneDDS><Domain><General><Interfaces>'
  for interface in "$@"; do
    cyclonedds_uri+="<NetworkInterface name=\"$interface\" priority=\"default\" multicast=\"true\"/>"
  done
  cyclonedds_uri+='</Interfaces></General></Domain></CycloneDDS>'
  export CYCLONEDDS_URI="$cyclonedds_uri"
}

# Arms the robot
function arm {
  ros2 action send_goal -f "$NAMESPACE"/go2_control/arm dua_interfaces/action/Arm "{}"
}

# Disarms the robot
function disarm {
  ros2 action send_goal -f "$NAMESPACE"/go2_control/disarm dua_interfaces/action/Disarm "{}"
}

# Performs a special action
function action {
  if [[ $# -ne 1 ]]; then
    echo >&2 "Usage: action ACTION_ID"
    return 1
  fi

  ros2 service call "$NAMESPACE"/go2_control/actions unitree_go/srv/Action "{action: {id: $1}}"
}
