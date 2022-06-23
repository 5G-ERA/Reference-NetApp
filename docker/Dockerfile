FROM ros:galactic-ros-base

RUN apt-get update

RUN apt-get install -y python3-pip 

ENV ROS2_5G_ERA_MMDET_PATH=/root/mmdetection

RUN cd /root/ \
    && mkdir -p dev_ws/src

COPY ./set_environment.sh /root/dev_ws 

COPY ./install_dependencies.sh /root/dev_ws 

COPY ./src /root/dev_ws/src

RUN cd /root/dev_ws \
    && rosdep update \
    && bash install_dependencies.sh 

RUN . /opt/ros/galactic/setup.sh \
    && cd /root/dev_ws \
    && colcon build

COPY ./docker/run.sh /root/

COPY ./assets/ /root/assets

ENV ROS2_5G_ERA_ASSETS_PATH=/root/assets

CMD ["/bin/bash", "/root/run.sh"]
