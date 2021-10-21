FROM ros:foxy

SHELL ["/bin/bash", "-c"]

# install dependencies
RUN apt-get update && apt-get install -y \
        ros-foxy-can-msgs \
        git \
        python3.8 \
        python3-pip \
        && rm -rf /var/lib/apt/lists/*


# copy files
RUN mkdir -p /dev_ws/src
WORKDIR /dev/src
COPY . /dev_ws/src/can_reader
WORKDIR /dev_ws

# install python dependences
RUN pip install -r /dev_ws/src/can_reader/requirements.txt
RUN . /opt/ros/foxy/setup.bash && colcon build

RUN . /dev_ws/install/setup.bash

# TODO: automatically source 
# TODO: find out why reader is not publishing?


# default command
CMD ["bash"]

# from: https://tuw-cpsg.github.io/tutorials/docker-ros/