FROM osrf/ros:melodic-desktop-bionic

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-melodic-desktop-full=1.4.1-0* \
    && rm -rf /var/lib/apt/lists/*

# install bash because we need to run a bash script
RUN apt-get install -y bash

# copy shell scripts to start ROS master and start data collection
COPY new_entrypoint.sh /
COPY bag2pcd.sh /

ENTRYPOINT [ "/bin/bash" ]
CMD ["/new_entrypoint.sh"]