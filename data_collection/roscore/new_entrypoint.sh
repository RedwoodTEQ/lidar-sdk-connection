#!/bin/bash

# Logging function from https://stackoverflow.com/questions/48086633/simple-logging-levels-in-bash/48087251
declare -A levels=([DEBUG]=0 [INFO]=1 [WARN]=2 [ERROR]=3)
script_logging_level="INFO"

logThis() {
    local log_message=$1
    local log_priority=$2

    #check if level exists
    [[ ${levels[$log_priority]} ]] || return 1

    #check if level is enough
    (( ${levels[$log_priority]} < ${levels[$script_logging_level]} )) && return 2

    #log here
    echo "${log_priority} : ${log_message}"
}


logThis "ROS core entrypoint loaded..." "INFO"
logThis "{$$}" "INFO"

collection_log() {
    while :; do
        cd /sharedstorage
        du -h *.bag
        du -hs .
        sleep 30
    done
}

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

collection_log &
collection_log_pid=$!

. /opt/ros/melodic/setup.sh && \
roscore -p 11311 &
. /opt/ros/melodic/setup.sh && \
cd /sharedstorage && \
rosbag record --split --duration=30 /rslidar_points -O rslidar_points &

child=$!
wait "$child"
kill $collection_log_pid



