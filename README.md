Installare docker: https://dev.to/elalemanyo/how-to-install-docker-and-docker-compose-on-raspberry-pi-1mo

docker run --network host --privileged -v "/dev/ttyACM0:/dev/ttyACM0" -v "/home/pi/serial/:/root/ros/" -it noetic-ros-base-erl /bin/bash

Per fare lampeggiare il led: rostopic pub toggle_led std_msgs/Empty --once
roslaunch rosserial_python serial_node.launch
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
rostopic echo cmd_vel

Ros.h arduino: https://www.youtube.com/watch?v=6F5mNGdBhlU

# This is an auto generated Dockerfile for ros:ros-base
# generated from docker_images/create_ros_image.Dockerfile.em
FROM ros:noetic-ros-core-focal

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    python3-rosdep \
    python3-rosinstall \
    python3-vcstools \
    && rm -rf /var/lib/apt/lists/*

# bootstrap rosdep
RUN rosdep init && \
  rosdep update --rosdistro $ROS_DISTRO

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-ros-base=1.5.0-1* \
    && rm -rf /var/lib/apt/lists/*

# ERL - personalization
RUN apt-get update && apt-get install --no-install-recommends -y \
    htop \
    tree \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*
RUN pip3 install pyserial
