# MA Adrian Scholze



## System Requirements

- Ubuntu 24.04.2 LTS
- ROS2 Jazzy Desktop (sudo apt install ros-jazzy-desktop)
- python 3.12
- python venv (sudo apt install python3.12-venv)
- julia 1.11.6 (curl -fsSL https://install.julialang.org | sh)
- colcon (sudo apt install colcon)
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
- git clone git@github.com:XPhantomad/Context-Role-Oriented-Transport-Chain-Swarm.git
- cd Context-Role-Oriented-Transport-Chain-Swarm
- chmod +x ./startup.sh

### Simulation
- copy the content of the rosWorkspace Folder in your ros_ws directory
- in the directory ros_ws:
    - source /opt/ros/jazzy/setup.bash
    - in src/argos3-ros2-bridge/CMakeLists.txt comment out line 60: add_subdirectory(plugins) for the first build
    - colcon build --packages-select argos3_ros2_bridge
    - uncomment line 60 and build it again
    - source install/setup.bash
    - argos3 -c bridge_example.argos
    - simulation should be ready

### Swarm-Element-Loop
- in terminal enter:
    - julia
    - import Pkg 
    - Pkg.add("Sockets")
    - Pkg.add("JSON")
    - Pkg.add("Parameters")

### Single-Robot-Loop
- in runtimemodel and messages folder setup venv:
    - open terminal and create python venv: python3 -m venv ./
    - source bin/activate
    - run: python3 main.py
    - install required pip packages (pip install pyyaml numpy pyecore)
    - quit application

### Webapp
- in webapp folder setup venv:
    - open terminal and create python venv: python3 -m venv ./
    - source bin/activate
    - run: python3 swarmDisplay.py
    - install required pip packages (pip install -r requirements.txt
    - quit application

## Run Application
- in /ros_ws run: 
    - source install/setup.bash
    - argos3 -c bridge_example.argos
    - start simulation by clicking the **play** button
- in webapp run (absolutely necessary before starting the robots):
    - source bin/activate
    - python3 swarmDisplay.py
    - connect to Webapp via link

- ./startup.sh [footbot-name] (e.g. fb_0)
- current simulation has red light as prey, orange ligth as nest and fb_1 to fb_6 for the Chain Task
- so start the command above for all three robots in seperate terminals 
- wait until robot is driving before starting the next one!! (otherwise tmux is confused) 
- left pane shows output from Swarm Element Loop
- right top Pane shows output from Single Robot Loop
- right bottom Pane shows output from Messages Component

- stop Application:
    - Strg+b d  (to detach from tmux)
    - tmux kill-server

## Demo

Short Teaser:

[![Watch the video](https://git-st.inf.tu-dresden.de/stgroup/student-projects/2025/ma-adrian-scholze/-/blob/main/crom-models/thumbnailTeaser.png)](https://youtu.be/6gFN5Zp4-fo)

Explanation 1:

[![Watch the video](https://git-st.inf.tu-dresden.de/stgroup/student-projects/2025/ma-adrian-scholze/-/blob/main/crom-models/thumbnailTeaser.png)](https://youtu.be/4wl4qiH0HfQ)

Explanation 2:

[![Watch the video](https://git-st.inf.tu-dresden.de/stgroup/student-projects/2025/ma-adrian-scholze/-/blob/main/crom-models/thumbnailTeaser.png)](https://youtu.be/RyUZocn-PCA)
