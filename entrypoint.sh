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
echo " ROS_MASTER_URI  : ${ROS_MASTER_URI:-http://localhost:11311}"
echo " ROS_IP          : ${ROS_IP:-non impostato}"
echo " ROS_HOSTNAME    : ${ROS_HOSTNAME:-non impostato}"
echo " MISSION_DIR     : ${MISSION_DIR:-/home/usv/missions}"
echo " DRONE_ID        : ${DRONE_ID:-1 (default)}"
echo "-----------------------------------------"
echo " NAVDATA_ADDR    : ${NAVDATA_SERVICE_ADDR:-navdata_service:60001 (default)}"
echo " COMMANDVEL_ADDR : ${COMMANDVEL_SERVICE_ADDR:-commandvel_service:60005 (default)}"
echo " GETCONTROL_ADDR : ${GETCONTROL_SERVICE_ADDR:-getcontrol_service:60004 (default)}"
echo " SENDMISSION_ADDR: ${SENDMISSION_SERVICE_ADDR:-sendmission_service:60008 (default)}"
echo "========================================="

# Costruisce gli argomenti roslaunch dalle env var
LAUNCH_ARGS=""
[ -n "$DRONE_ID" ]               && LAUNCH_ARGS="$LAUNCH_ARGS drone_id:=$DRONE_ID"
[ -n "$MISSION_DIR" ]            && LAUNCH_ARGS="$LAUNCH_ARGS mission_dir:=$MISSION_DIR"
[ -n "$NAVDATA_SERVICE_ADDR" ]   && LAUNCH_ARGS="$LAUNCH_ARGS navdata_service_addr:=$NAVDATA_SERVICE_ADDR"
[ -n "$COMMANDVEL_SERVICE_ADDR" ] && LAUNCH_ARGS="$LAUNCH_ARGS commandvel_service_addr:=$COMMANDVEL_SERVICE_ADDR"
[ -n "$GETCONTROL_SERVICE_ADDR" ] && LAUNCH_ARGS="$LAUNCH_ARGS getcontrol_service_addr:=$GETCONTROL_SERVICE_ADDR"
[ -n "$SENDMISSION_SERVICE_ADDR" ] && LAUNCH_ARGS="$LAUNCH_ARGS sendmission_service_addr:=$SENDMISSION_SERVICE_ADDR"

# Esegui il comando passato (default: roslaunch dal CMD del Dockerfile)
exec "$@" $LAUNCH_ARGS
