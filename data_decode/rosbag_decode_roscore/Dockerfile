FROM osrf/ros:melodic-desktop-bionic
# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-melodic-desktop-full=1.4.1-0* \
    && rm -rf /var/lib/apt/lists/*
COPY new_entrypoint.sh /
COPY rosbag_decode.py /

CMD ["/bin/sh", "/new_entrypoint.sh"]