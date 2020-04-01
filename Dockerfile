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
                numba \
                matplotlib \
                zmq \
                pyzmq \
                Pillow \
                gym \
                protobuf \
                pyyaml \
                msgpack==0.6.2

RUN git clone https://github.com/f1tenth/f1tenth_gym

RUn cd f1tenth_gym && \
    mkdir -p build && \
    cd build && \
    cmake .. && \
    make

RUN cp build/sim_request_pb2.py gym/

RUN pip install -e gym/

ENTRYPOINT ["/bin/bash"]


# CMD ["roslaunch", "package file.launch"]