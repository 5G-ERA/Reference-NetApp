#!/bin/bash
set -e

# setup ros environment
source "/root/ros_ws/devel/setup.bash"

exec rosrun era_5g_netapp_reference_client node.py