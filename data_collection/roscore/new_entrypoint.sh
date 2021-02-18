#!/bin/sh

. /opt/ros/melodic/setup.sh && \
roscore -p 11311 &
. /opt/ros/melodic/setup.sh && cd /storage && rosbag record --split --duration=30 /rslidar_packets /rslidar_packets_difop -O rslidar_packet