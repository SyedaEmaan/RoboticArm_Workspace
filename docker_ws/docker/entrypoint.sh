#!/bin/bash 
set -e 

source /opt/ros/humble/setup.bash 
source /usr/share/gazebo-11/setup.sh

if [ -f /home/rosdev/arm_ws/install/setup.bash ]; then 
	source /home/rosdev/arm_ws/install/setup.bash
fi

exec "$@"


