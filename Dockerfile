FROM ros:melodic-robot-bionic

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
                       vim \
                       ros-melodic-ackermann-msgs \
                       ros-melodic-map-server \
                       ros-melodic-genpy


RUN cp -r /usr/include/eigen3/Eigen /usr/include

RUN git clone https://github.com/protocolbuffers/protobuf.git && \
    cd protobuf && \
    git checkout tags/v3.8.0 && \
    git submodule update --init --recursive && \
    ./autogen.sh && \
    ./configure && \
    make -j8 && \
    make install && \
    ldconfig && \
    make clean && \
    cd .. && \
    rm -r protobuf

RUN pip install --upgrade pip

RUN pip install numpy==1.16.0 \
                scipy==1.2.0 \
                zmq \
                pyzmq \
                Pillow \
                gym \
                protobuf==3.8.0 \
                pyyaml \
                llvmlite==0.31.0 \
                numba==0.47.0


# RUN git clone https://github.com/f1tenth/f1tenth_gym
RUN mkdir /f1tenth_gym
COPY ./f1tenth_gym /f1tenth_gym

RUN cd f1tenth_gym && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make

RUN cd f1tenth_gym && \
    cp ./build/sim_requests_pb2.py ./gym/ && \
    pip install -e gym/

RUN /bin/bash -c "source /opt/ros/melodic/setup.bash; mkdir -p catkin_ws/src; cd catkin_ws; catkin_make"

RUN mkdir /catkin_ws/src/f1tenth_gym_ros

COPY . /catkin_ws/src/f1tenth_gym_ros

RUN /bin/bash -c "source /opt/ros/melodic/setup.bash; cd catkin_ws; catkin_make; source devel/setup.bash"


CMD ["/catkin_ws/src/f1tenth_gym_ros/start.sh"]

# CMD ["roslaunch", "package file.launch"]
