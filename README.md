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

## Quickstart with Docker

To see a running simulation with 4 robots, we provide an easy to use Docker Image containing the overall system. It can be started using the run.sh or run.but scripts on Linux and Windows respectively. The only requirement for this is Docker to be installed.

For a closer investigation (e.g., to reproduce the measurements from the paper), you need to follow the installation instructions below. We included the Dockerfile in the rosWorkspace folder as a reference, as the installation is quite complicated.

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
### Run Simulation 
- in /ros_ws run: 
    - ```source install/setup.bash```
    - ```argos3 -c bridge_example.argos```
    - start simulation by clicking the **play** button

### Run Robots via startup script
- in main folder run:
    - ```source ~/ros_ws/install/setup.bash```
    - ```source ./startup/bin/activate```
    - ```python3 ./startup/automatedStartup.py``` (adjust number of iterations depending on the number of robots)
- runs Webapp and the Robots with its 3 Components successive
- open Dashboard: http://localhost:5000/

### Run Robots one by one (alternative to startup script --> seperate output for each Robot)
- [run simulation](#run-simulation)
- in webapp run (absolutely necessary before starting the robots):
    - ```source bin/activate```
    - ```python3 swarmDisplay.py```
    - connect to Webapp via link

- ```./startup.sh [footbot-name]``` (e.g. fb_0)
- current simulation has red light as **Prey**, orange ligth as **Nest** and fb_0 to fb_6 for the Chain Task
- so start the command above for all 7 robots in seperate terminals 
- wait until robot is driving before starting the next one!! (otherwise tmux is confused) 
- left pane shows output from Swarm Element Loop
- right top Pane shows output from Single Robot Loop
- right bottom Pane shows output from Messages Component

- stop Application:
    - Strg+b d  (to detach from tmux)
    - ```tmux kill-server```

### Run Tests
- in main folder run:
    - ```source ~/ros_ws/install/setup.bash```
    - ```source ./startup/bin/activate```
- run Tests with (e.g.):
    - ```python3 RobotWithLoadDetected.py```
- Test: PreyDetected.py runs Test only once, because this Action can not be reseted without restarting the application
- Execution Times are stored in associated files (e.g. time_RobotWithLoadDetected.txt)
- After running all tests the ExecutionTime can be plotted with ./ExecutionTimeMeasurements/timePlotToolFSET_4Cases.py
- For that execute in main folder:
    - ```source ./ExecutionTimeMeasurements/bin/activate```
    - ```python3 ./ExecutionTimeMeasurements/timePlotToolFSET_4Cases.py```

### Create SEL-SRL-MSG Execution Time Plot
- uncomment lines 37 and 77-79 in messages/main.py
- uncomment lines 95-98 and 135-139 in runtimemodel/main.py
- run simulation ([here](#run-simulation)) and execute automatedStartup.py as mentioned before [here](#run-robots-via-startup-script)
- execution times of the SEL will always be stored in time.txt
- execution times of SRL will be stored in timeSRL.txt
- execution times of Messages Component will be stored in timeMSG.txt
- after running the application for a while, stop
- plot times with ```python3 ExecutionTimeMeasurement/timePlotToolSEL_SRL_MSG.py``` executed from the main folder

### Create Plot for Scalability
- run application ([run simulation](#run-simulation) & [startup](#run-robots-via-startup-script)) for different numbers of robots ([change robot number](#change-robot-number-in-simulation))
- after each run, save the times.txt with another name (e.g. timeXRobots.txt)
- after having 4 measurements, plot them with ```python3 ExecutionTimeMeasurement/timePlotToolScalability.py``` (change names of the used timeXRobots.txt files)


## Change Robot Number in Simulation
- in your copied ROS Workspace under ros_ws change the following parameters
- in ros_ws_/bridge_example.argos:
    - change ```<position>``` of the Prey light
    - change the distributioin of robots by adjusting the ```<position>``` min and max Positions and the ```<entity>``` quantity
    ```html
    <light id="Prey"
            position="9,1,0.2"
            orientation="0,0,0"
            color="red"
            intensity="1.0"
            medium="leds" />
    ...   
    <distribute>
        <position method="uniform" min="-2.5,-2.0,0" max="11.0,4.0,0" />
        <orientation method="uniform" min="0,0,0" max="360,0,0" />
        <entity quantity="15" max_trials="100">
            <foot-bot id="fb_">
            <controller config="lrb" />
            </foot-bot>
        </entity>
        </distribute> 
    ``` 
- in ros_ws/src/argos3-ros2-bridge/plugins/loop_functions/foraging_loop_functions/foraging_loop_functions.cpp: change the positioning of the load (black circles) 
    - set the ```m_preyPosition(9,1),``` in line 9 to the same position as the Prey light 
    - in ros_ws run: ```colcon build --packages-select argos3_ros2_bridge```
    - run ```source install/setup.bash```
    - run ``` argos3 -c bridge_example.argos ``` to see if it worked
- in Contexts/swarmElementLoop/MAPE.jl adjust lines Explorartion Area (lines 450-451) to a new area (optimally around prey; otherwise, the robots need unduly long to find the prey and start forming a chain)
    ```julia
    # 0 Exploration
    areaPos1 = Position(5,0) 
    areaPos2 = Position(11,4) 
        if getRoles(robotSelf) === nothing
            ...
    ```


## Demo

Short Teaser:

[![Watch the video](https://git-st.inf.tu-dresden.de/stgroup/student-projects/2025/ma-adrian-scholze/-/blob/main/crom-models/thumbnailTeaser.png)](https://youtu.be/YxY3P1U7E8I)

Explanation 1:

[![Watch the video](https://git-st.inf.tu-dresden.de/stgroup/student-projects/2025/ma-adrian-scholze/-/blob/main/crom-models/thumbnailTeaser.png)](https://youtu.be/M2knKsVhV9w)

