FROM but5gera/netapp_base_mmcv_gpu:0.3.0

ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=Europe/Prague
ENV LANG C.UTF-8

COPY ros2/era_5g_object_detection /root/src/era_5g_object_detection

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
RUN apt-get update && apt-get install -y curl gnupg2 lsb-release software-properties-common
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS Humble
RUN apt-get update  \
    && apt-get upgrade -y \
    && apt-get install -y ros-humble-ros-base python3-argcomplete \
    ros-humble-cv-bridge git

COPY docker/ros2_object_detection/DEFAULT_FASTRTPS_PROFILES.xml /root/src/era_5g_object_detection

COPY docker/ros2_object_detection/start.sh /root/start.sh

RUN chmod +x /root/start.sh
ENTRYPOINT ["/root/start.sh"]
