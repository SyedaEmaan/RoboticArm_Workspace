#!/bin/bash 
set -e 

source /opt/ros/humble/setup.bash 
source /usr/share/gazebo-11/setup.sh

# Source EtherCAT driver workspace
if [ -f /home/rosdev/ethercat_ws/install/setup.bash ]; then
    source /home/rosdev/ethercat_ws/install/setup.bash
fi

if [ -f /home/rosdev/arm_ws/install/setup.bash ]; then 
	source /home/rosdev/arm_ws/install/setup.bash
fi

exec "$@"


