#!/bin/bash
parent_path=$( cd "$(dirname "${BASH_SOURCE[0]}")" ; pwd -P )

(
  cd "$parent_path" || exit 1
  xacro robot.urdf.xacro use_ballfeet:=true > ../magnecko_ballfeet.urdf
  xacro robot.urdf.xacro use_ballfeet:=false > ../magnecko.urdf
)