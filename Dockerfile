FROM ros:melodic-ros-base

ENV IM_IN_DOCKER Yes

RUN apt-get update --fix-missing && \
    apt-get install -y \
    python-pip

RUN apt-get install -y libzmq3-dev \
                       git \
                       build-essential \
                       autoconf \
                       libtool \
                       libeigen3-dev \
                       cmake \
                       vim

RUN cp -r /usr/include/eigen3/Eigen /usr/include

RUN git clone https://github.com/google/protobuf.git && \
    cd protobuf && \
    ./autogen.sh && \
    ./configure && \
    make -j8 && \
    make install && \
    ldconfig && \
    make clean && \
    cd .. && \
    rm -r protobuf

RUN pip install --upgrade pip

RUN pip install numpy \
                scipy \
                zmq \
                pyzmq \
                Pillow \
                gym \
                protobuf \
                pyyaml \

RUN git clone https://github.com/f1tenth/f1tenth_gym

RUn cd f1tenth_gym && \
    mkdir -p build && \
    cd build && \
    cmake .. && \
    make

RUN cp build/sim_request_pb2.py gym/

RUN pip install -e gym/

RUN source /opt/ros/melodic/setup.bash

RUN cd .. && \
    mkdir -p catkin_ws/src && \
    cd catkin_ws && \
    catkin_make

COPY . /catkin_ws/src

RUN catkin_make && \
    source devel/setup.bash


CMD ["roslaunch", "f1tenth_gym_ros gym_bridge.launch"]


# CMD ["roslaunch", "package file.launch"]