#!/bin/bash

echo "ROS core entrypoint loaded..."
echo $$

_term () {
    echo "Caught SIGTERM signal!"
    kill -TERM "$child" 2>/dev/null
    wait "$child"
    archive
    exit 0
}

archive () {
    echo "Compressing and archiving data before exiting."
    echo "Please wait..."
    cd /sharedstorage
    echo "1234" >> /sharedstorage/archive.txt
    rm -rf pcd_out/
    for f in *.bag ; do
            . /opt/ros/melodic/setup.sh && rosrun pcl_ros bag_to_pcd ${f} /rslidar_points pcd_out
    done
    i=0
    while [[ -f "lidar${i}.pcd.gz" ]]; do
            i=$((i+1))
    done
    if [[ -d /sharedstorage/pcd_out/ ]]; then
        tar -zcvf lidar${i}.pcd.gz /sharedstorage/pcd_out/
    fi
}

trap _term SIGTERM TERM
echo "ABCDEF"

. /opt/ros/melodic/setup.sh && \
roscore -p 11311 &
. /opt/ros/melodic/setup.sh && \
cd /sharedstorage && \
rosbag record --split --duration=30 /rslidar_points -O rslidar_points &

child=$!
wait "$child"



