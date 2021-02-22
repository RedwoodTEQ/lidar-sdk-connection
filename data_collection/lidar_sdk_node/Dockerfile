FROM osrf/ros:melodic-desktop-bionic

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-melodic-desktop-full=1.4.1-0* \
    && rm -rf /var/lib/apt/lists/*

# install pcap & install protobuf
RUN apt-get update && apt-get install -y libyaml-cpp-dev libpcap-dev \
    libprotobuf-dev protobuf-compiler

# download rslidar_sdk via git, build and run
RUN . /opt/ros/melodic/setup.sh && cd /home && git clone https://github.com/RoboSense-LiDAR/rslidar_sdk.git \
    && cd rslidar_sdk && git submodule init && git submodule update && mkdir build \
    && cd build && cmake .. && make -j4

# copy Lidar configuration file
COPY config.yaml /home/rslidar_sdk/config/

# copy shell script to run the Lidar sdk
COPY new_node_entrypoint.sh /

# set environment variable for ROS master to send data to
ENV ROS_MASTER_URI="http://docker_roscore_master:11311/"

ENTRYPOINT [ "/bin/sh" ]
CMD [ "/new_node_entrypoint.sh" ]