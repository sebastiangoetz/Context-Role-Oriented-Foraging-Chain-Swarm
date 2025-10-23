FROM osrf/ros:jazzy-desktop-full

# Change the default shell to Bash
SHELL [ "/bin/bash" , "-c" ]

# Install Requirements
RUN apt-get update && apt-get install -y git colcon cmake libfreeimage-dev libfreeimageplus-dev freeglut3-dev libxi-dev libxmu-dev liblua5.3-dev lua5.3 doxygen graphviz libgraphviz-dev asciidoc

RUN cd /usr/lib/x86_64-linux-gnu \
    && ln -s libglut.so.3.12.0 libglut.so.3

RUN git clone https://github.com/sebastiangoetz/Context-Role-Oriented-Foraging-Chain-Swarm.git

# Install ARGoS3
RUN cd Context-Role-Oriented-Foraging-Chain-Swarm \
    && cd Dependencies \
    && apt-get install -y ./argos3_simulator-3.0.0-x86_64-beta59.deb

# Install GSL for ARGoS3-ROS2-Bridge
RUN apt-get install -y libgsl-dev

RUN cd Context-Role-Oriented-Foraging-Chain-Swarm \
    && cd Dependencies \
    && tar -zxf gsl-latest.tar.gz \
    && cd gsl-2.8 \
    && mkdir /root/gsl \
    && ./configure --prefix=/root/gsl \
    && make \
    && make check \
    && make install 

ENV LD_LIBRARY_PATH="/root/gsl/lib:$LD_LIBRARY_PATH"

# Build ARGoS3-ROS2-Bridge
RUN ln -s Context-Role-Oriented-Foraging-Chain-Swarm/rosWorkspace ros_ws 

RUN cd ros_ws \
    && chmod +x build.sh \
    && ./build.sh

RUN cd Context-Role-Oriented-Foraging-Chain-Swarm \
    && git reset --hard
  
RUN cd ros_ws \
    && chmod +x build.sh \
    && ./build.sh \
    && source install/setup.bash \
    && argos3 -c bridge_example.argos