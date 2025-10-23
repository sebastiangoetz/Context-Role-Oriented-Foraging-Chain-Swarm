# MA Adrian Scholze



## System Requirements

- Ubuntu 24.04.2 LTS
- ROS2 Jazzy Desktop (sudo apt install ros-jazzy-desktop)
- python 3.12
- python venv (```sudo apt install python3.12-venv```)
- julia 1.11.6 (```curl -fsSL https://install.julialang.org | sh```)
- colcon (```sudo apt install colcon```)
- rqt
- tmux 3.4 (```sudo apt-get install tmux```)

### Install Argos3 Simulator
- Prerequisites:
  - ```sudo apt-get install cmake libfreeimage-dev libfreeimageplus-dev qt5-default freeglut3-dev libxi-dev libxmu-dev liblua5.3-dev lua5.3 doxygen graphviz libgraphviz-dev asciidoc ```
  - Freeglut 3: Problem with libglut.so.3.12
    1. Install the freeglut3-dev package with:
       
       ```sudo apt install freeglut3-dev```
    2. Change to the /usr/lib/x86_64-linux-gnu directory by enter: 

       ```cd /usr/lib/x86_64-linux-gnu```
    3. Now create a new symlink with name libglut.so.3 which points to libglut.so.3.12.0:
       
        ``` ln -s libglut.so.3.12.0 libglut.so.3```
- Download argos3_simulator-3.0.0-x86_64-beta59.deb (in Folder *DownloadedPackages*)
- ```sudo apt install ./argos3_simulator-3.0.0-x86_64-beta59.deb```

Verify Installation
- ```git clone https://github.com/ilpincy/argos3-examples.git``` to Documents
- compile and test with instructions from https://github.com/ilpincy/argos3-examples
- ```argos3 -c ./experiments/diffusion_1.argos``` shuold throw no error

### Install GSL for Argos3-ROS2-Bridge
- https://coral.ise.lehigh.edu/jild13/2016/07/11/hello/
    - ```sudo apt-get install libgsl-dev```
    - Download gsl-latest.tar.gz (in Folder *DownloadedPackages*)
    - ```tar -zxvf gsl-*.*.tar.gz```
    - ```cd gsl-1.7```
    - ```mkdir /home/yourname/gsl```
    - ```./configure --prefix=/home/yourname/gsl```
    - ```make``` (takes a while)
    - ```make check```
    - ```make install```
    - ```'export LD_LIBRARY_PATH=*/path/to/library*:$LD_LIBRARY_PATH' >> ~/.bashrc ```

## Install Application
- ```git clone git@github.com:XPhantomad/Context-Role-Oriented-Transport-Chain-Swarm.git ```
- ```cd Context-Role-Oriented-Transport-Chain-Swarm```
- ```chmod +x ./startup.sh```

### Simulation
- copy the content of the rosWorkspace Folder in your ```ros_ws``` directory
- in the directory ```ros_ws```:
    - ```source /opt/ros/jazzy/setup.bash```
    - in src/argos3-ros2-bridge/CMakeLists.txt comment out line 60: ```add_subdirectory(plugins)``` for the first build
    - ```colcon build --packages-select argos3_ros2_bridge```
    - uncomment line 60 and build it again
    - ```source install/setup.bash```
    - ```argos3 -c bridge_example.argos```
    - simulation should be ready

### Swarm-Element-Loop
- in terminal enter:
    - ```julia```
    - import Pkg 
    - Pkg.add("Sockets")
    - Pkg.add("JSON")
    - Pkg.add("Parameters")

### Startup 
- in startup folder setup venv:
    - open terminal and create python venv: ```python3 -m venv ./```
    - ```source bin/activate```
    - install required pip packages (```pip install -r requirements.txt```)


### Single-Robot-Loop (requried only for single Robot startup)
- in runtimemodel and messages folder setup venv:
    - open terminal and create python venv: ```python3 -m venv ./```
    - ```source bin/activate```
    - install required pip packages (```pip install -r requirements.txt```)
    - run: ```python3 main.py```
    - quit application

### Webapp (requried only for single Robot startup)
- in webapp folder setup venv:
    - open terminal and create python venv: ```python3 -m venv ./```
    - ```source bin/activate```
    - install required pip packages (```pip install -r requirements.txt```)
    - run: ```python3 swarmDisplay.py```
    - quit application

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

[![Watch the video](https://git-st.inf.tu-dresden.de/stgroup/student-projects/2025/ma-adrian-scholze/-/blob/main/crom-models/thumbnailTeaser.png)](https://youtu.be/6gFN5Zp4-fo)

Explanation 1:

[![Watch the video](https://git-st.inf.tu-dresden.de/stgroup/student-projects/2025/ma-adrian-scholze/-/blob/main/crom-models/thumbnailTeaser.png)](https://youtu.be/4wl4qiH0HfQ)

Explanation 2:

[![Watch the video](https://git-st.inf.tu-dresden.de/stgroup/student-projects/2025/ma-adrian-scholze/-/blob/main/crom-models/thumbnailTeaser.png)](https://youtu.be/RyUZocn-PCA)
