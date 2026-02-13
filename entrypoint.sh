#!/bin/bash
set -e

# Source ROS Noetic
source /opt/ros/noetic/setup.bash

# Source workspace
source /catkin_ws/devel/setup.bash

# Log configurazione di rete ROS
echo "========================================="
echo " edge_mission_controller container"
echo "========================================="
echo " ROS_MASTER_URI : ${ROS_MASTER_URI:-http://localhost:11311}"
echo " ROS_IP         : ${ROS_IP:-non impostato}"
echo " ROS_HOSTNAME   : ${ROS_HOSTNAME:-non impostato}"
echo " MISSION_DIR    : ${MISSION_DIR:-/home/usv/missions}"
echo "========================================="

# Esegui il comando passato (default: roslaunch dal CMD del Dockerfile)
exec "$@"
