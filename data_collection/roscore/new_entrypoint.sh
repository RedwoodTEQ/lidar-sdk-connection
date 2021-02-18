#!/bin/sh

. /opt/ros/melodic/setup.sh && \
roscore -p 11311 &
. /opt/ros/melodic/setup.sh && cd /storage && rosbag record --split --duration=30 /rslidar_points -O rslidar_points