#!/bin/bash

source ~/ros_ws/install/setup.bash
ros2 topic pub --once /Prey/cmd_led argos3_ros2_bridge/msg/Led "{color: "red", mode: "SINGLE", index: 12}"