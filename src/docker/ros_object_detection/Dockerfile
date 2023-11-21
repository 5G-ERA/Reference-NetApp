FROM but5gera/netapp_base_mmcv_gpu:0.2.0-ubuntu20.04

ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=Europe/Prague
ENV LANG C.UTF-8

RUN pip install --upgrade pip

COPY python/era_5g_object_detection_common /root/era_5g_object_detection_common

RUN cd /root/era_5g_object_detection_common \
    && pip install -r requirements.txt \
    && pip install .

COPY python/era_5g_object_detection_standalone /root/era_5g_object_detection_standalone

RUN cd /root/era_5g_object_detection_standalone \
    && pip install -r requirements.txt \
    && pip install .

ENV NETAPP_MODEL_VARIANT=yolov3_mobilenet
ENV NETAPP_TORCH_DEVICE=cuda:0

# Add the ROS package repository to apt sources list
RUN apt-get update && apt-get install -y curl gnupg2 lsb-release
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Install ROS Noetic
RUN apt-get update && apt-get install -y ros-noetic-ros-base ros-noetic-cv-bridge git

# Initialize ROS and source the setup file
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc

# Create a workspace directory
RUN mkdir -p /root/catkin_ws/src

# Set the workspace as the working directory
WORKDIR /root/catkin_ws

COPY ros1/era_5g_object_detection /root/catkin_ws/src/era_5g_object_detection

# Build the workspace
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Source the workspace setup file
RUN echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc

COPY docker/ros_object_detection/start.sh /root/start.sh

RUN chmod +x /root/start.sh
ENTRYPOINT ["/root/start.sh"]