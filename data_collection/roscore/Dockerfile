FROM osrf/ros:melodic-desktop-bionic

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-melodic-desktop-full=1.4.1-0* \
    && rm -rf /var/lib/apt/lists/*

# copy shell script to start ROS master and start data collection
COPY new_entrypoint.sh /

ENTRYPOINT [ "/bin/sh" ]
CMD ["/new_entrypoint.sh"]