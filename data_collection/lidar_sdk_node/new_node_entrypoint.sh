#!/bin/sh
echo $$

_term () {
    echo "Caught SIGTERM signal!"
    kill -TERM "$child" 2>/dev/null
    wait "$child"
    exit 0
}

trap _term TERM

. /opt/ros/melodic/setup.sh && \
echo $ROS_MASTER_URI && \
./home/rslidar_sdk/build/rslidar_sdk_node &

child=$!
wait "$child"