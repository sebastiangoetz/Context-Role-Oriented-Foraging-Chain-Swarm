#!/bin/bash
source /opt/ros/jazzy/setup.bash
rm -r build install log
colcon build
mv src/argos3-ros2-bridge/CMakeLists.txt src/argos3-ros2-bridge/CMakeLists-Step1.txt 
mv src/argos3-ros2-bridge/CMakeLists-Step2.txt src/argos3-ros2-bridge/CMakeLists.txt 
colcon build
source install/setup.bash