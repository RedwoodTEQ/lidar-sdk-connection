#!/bin/sh

. /opt/ros/melodic/setup.sh && roscore -p 11311 &
. /opt/ros/melodic/setup.sh && cd /storage && rosbag play rslidar_packet_0.bag rslidar_packet_1.bag rslidar_packet_2.bag rslidar_packet_3.bag rslidar_packet_4.bag rslidar_packet_5.bag rslidar_packet_6.bag rslidar_packet_7.bag --topics /rslidar_packet &
. /opt/ros/melodic/setup.sh && cd /storage && rosbag record --split --duration=30 /rslidar_points -O rslidar_points