#!/bin/sh

. /opt/ros/melodic/setup.sh && cd /storage && rosbag play rslidar_points*.bag
. /opt/ros/melodic/setup.sh && rostopic echo /rslidar_points > /output/point_cloud.out