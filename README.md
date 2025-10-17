# Using Context Role-oriented Programming for Swarms to Alleviate the Micro-Macro Problem

This repository contains the code of our implmentation of a Foraging Chain Swarm in ARGoS3 using Context-Role-Oriented Programming.

The overall system is comprised of multiple subsystems, which have to be installed and started independently. 

We use ROS2 Jazzy, which is only easy to install if you follow the version restrictions for the underlying Linux. 
In terms of performance, we encourage to install <b>Ubuntu 24.04 (not 24.10 or the latest version)</b> bare-bones, i.e., not in a virtual machine.
Using a virtual machine is possible, but degrades performance considerably.

A detailed description of how to install ROS2 on your system can be found [here](https://docs.ros.org/en/jazzy/Installation.html).

Besides ROS2, we use ARGoS3 as simulator. Instructions on how to install the simulator can be found below.

Our prototype is comprised of five main parts in the following subdirectories:

- Contexts: contains the implementation of the <b>Swarm Element Loop</b> using [Contexts.jl](https://github.com/cgutsche/Contexts.jl)
- rosWorkspace: contains the implementation of the [ROS2-ARGoS3 bridge](https://github.com/einstein07/collective-decision-making-argos-ros2) including the UI extensions for our example (e.g., showing the names of the robots in ARGoS3)
- runtime model: contains the single robot loop implemented in Python using PyEcore for the runtime model
- messages: contains the messages component responsible to process the monitored sensor values from the robots and to pass them to the swarm element loop
- webapp: contains the dashboard to observe the overall system

The runtime model, messages and webapp components all use Python and require an own Python Environment to install the required dependencies.
The Contexts component requires Julia to be installed.

Due to the multitude of components we could not provide a ready-to-use image or Docker files. Find the detailed installation and startup instructions below.

## System Requirements

- Ubuntu 24.04.2 LTS
- ROS2 Jazzy Desktop (sudo apt install ros-jazzy-desktop)
- colcon (sudo apt install colcon)
- python 3.12
- python venv (sudo apt install python3.12-venv)
- julia 1.11.6 (curl -fsSL https://install.julialang.org | sh)
- rqt
- tmux 3.4 (sudo apt-get install tmux)

### Install Argos3 Simulator
- Prerequisites:
  - sudo apt-get install cmake libfreeimage-dev libfreeimageplus-dev \
  qt5-default freeglut3-dev libxi-dev libxmu-dev liblua5.3-dev \
  lua5.3 doxygen graphviz libgraphviz-dev asciidoc 
  - Freeglut 3: Problem with libglut.so.3.12
    1. Install the freeglut3-dev package with:
       
       sudo apt install freeglut3-dev
    2. Change to the /usr/lib/x86_64-linux-gnu directory by enter: 

       cd /usr/lib/x86_64-linux-gnu
    3. Now create a new symlink with name libglut.so.3 which points to libglut.so.3.12.0:
       
       ln -s libglut.so.3.12.0 libglut.so.3
- Download argos3_simulator-3.0.0-x86_64-beta59.deb (in Folder *DownloadedPackages*)
- sudo apt install ./argos3_simulator-3.0.0-x86_64-beta59.deb

Verify Installation
- git clone https://github.com/ilpincy/argos3-examples.git to Documents
- compile and test with instructions from https://github.com/ilpincy/argos3-examples
- argos3 -c ./experiments/diffusion_1.argos shuold throw no error

### Install GSL for Argos3-ROS2-Bridge
- https://coral.ise.lehigh.edu/jild13/2016/07/11/hello/
    - sudo apt-get install libgsl-dev
    - Download gsl-latest.tar.gz (in Folder *DownloadedPackages*)
    - tar -zxvf gsl-*.*.tar.gz
    - cd gsl-1.7
    - mkdir /home/yourname/gsl
    - ./configure --prefix=/home/yourname/gsl
    - make (takes a while)
    - make check
    - make install
    - 'export LD_LIBRARY_PATH=*/path/to/library*:$LD_LIBRARY_PATH' >> ~/.bashrc 

## Install Application

### Simulation
- create a symlink ros_ws in your home directory pointing the the rosWorkspace directory in this repository
  - cd ~
  - ln -s path-to-repo/rosWorkspace ros_ws
- in the directory ros_ws:
    - source /opt/ros/jazzy/setup.bash (if not already part of your .bashrc)
    - in src/argos3-ros2-bridge/CMakeLists.txt comment out line 60: add_subdirectory(plugins) for the first build
    - colcon build 
    - uncomment line 60 and build it again
    - source install/setup.bash
    - argos3 -c bridge_example.argos
    - simulation should be ready

### Swarm-Element-Loop
- To install all required Julia packages, in terminal enter:
    - julia
    - import Pkg 
    - Pkg.add("Sockets")
    - Pkg.add("JSON")
    - Pkg.add("Parameters")
    - exit()

### Single-Robot-Loop
- in runtimemodel and messages folder setup venv:
    - open terminal and create python venv: python3 -m venv ./
    - source bin/activate
    - install required pip packages (pip install pyyaml numpy pyecore)

### Webapp
- in webapp folder setup venv:
    - open terminal and create python venv: python3 -m venv ./
    - source bin/activate
    - install required pip packages (pip install -r requirements.txt)
 
## Run Application
- in /ros_ws run: 
    - source install/setup.bash
    - argos3 -c bridge_example.argos
    - start simulation by clicking the **play** button
- in webapp run (absolutely necessary before starting the robots):
    - source bin/activate
    - python3 swarmDisplay.py
    - connect to Webapp via link
    - do not worry that the page is currently empty - boxes will appear as soon as you startup the robots

- ./startup.sh [footbot-name] (e.g. fb_0)
- current simulation has red light as prey, orange ligth as nest and fb_1 to fb_6 for the Chain Task
- so start the command above for all robots in seperate terminals 
- wait until robot is driving before starting the next one!! (otherwise tmux is confused) 
- left pane shows output from Swarm Element Loop
- right top Pane shows output from Single Robot Loop
- right bottom Pane shows output from Messages Component

- stop Application:
    - Strg+b d  (to detach from tmux)
    - tmux kill-server

- If you just start one robot, it will drive between Prey and Nest alone
- If you start multiple robots, the will form a chain

Here's how the running application looks like for 3 robots that successfully formed a foraging chain.

<img alt="foraging-chain" src="https://github.com/user-attachments/assets/b74e760d-5b4d-4cf0-b03c-bf71a49cc38f" />
