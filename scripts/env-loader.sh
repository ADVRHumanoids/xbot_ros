#!/bin/bash

echo "Loading env!" >> /tmp/ssh_ok.txt

source /opt/ros/kinetic/setup.bash
source /home/centauro/devel-superbuild/robotology-setup.bash

export ROS_MASTER_URI=http://10.24.4.100:11311/
export ROS_IP=10.24.4.77
export ROS_HOSTNAME=10.24.4.77

exec "$@"
