#!/bin/bash
cd || exit
source dev_ws/install/local_setup.bash
ros2 run era_5g_object_detection_standalone ml_service
