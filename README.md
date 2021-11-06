ROS autonomus differential wheel

ROS Autonomus car with ROS executed using Docker.

Installare docker: https://dev.to/elalemanyo/how-to-install-docker-and-docker-compose-on-raspberry-pi-1mo

docker run --network host --privileged -v "/dev/ttyACM0:/dev/ttyACM0" -v "/home/pi/serial/:/root/ros/" -it noetic-ros-base-erl /bin/bash

Per fare lampeggiare il led: rostopic pub toggle_led std_msgs/Empty --once
roslaunch rosserial_python serial_node.launch
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
rostopic echo cmd_vel

Ros.h arduino: https://www.youtube.com/watch?v=6F5mNGdBhlU

