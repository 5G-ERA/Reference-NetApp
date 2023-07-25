FROM ros:noetic-ros-base

RUN apt-get update \
    && apt-get install -y python3-pip ffmpeg libsm6 libxext6 ros-noetic-cv-bridge

COPY noetic_client/era_5g_client-0.2.0-py3-none-any.whl /root/era_5g_client-0.2.0-py3-none-any.whl

RUN pip install --upgrade pip wheel \
    && pip install /root/era_5g_client-0.2.0-py3-none-any.whl websocket-client

# workspace setup
RUN mkdir -p /root/ros_ws/src \
    && cd /root/ros_ws/src \
    && /bin/bash -c "source /opt/ros/noetic/setup.bash; catkin_init_workspace" \
    && cd .. \
    && /bin/bash -c "source /opt/ros/noetic/setup.bash; catkin_make" 

COPY noetic_client/era_5g_netapp_reference_client /root/ros_ws/src/era_5g_netapp_reference_client
COPY noetic_client/era_5g_msgs /root/ros_ws/src/era_5g_msgs

RUN cd /root/ros_ws/ \
    && /bin/bash -c "source devel/setup.bash; catkin_make" 

COPY ./noetic_client/start.sh /root/start.sh
CMD ["/root/start.sh"]