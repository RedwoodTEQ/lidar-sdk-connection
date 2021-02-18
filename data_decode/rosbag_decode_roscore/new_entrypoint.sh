#!/bin/sh

. /opt/ros/melodic/setup.sh && \
roscore -p 11311
# . /opt/ros/melodic/setup.sh && cd /storage && rosbag play rslidar_packet*.bag