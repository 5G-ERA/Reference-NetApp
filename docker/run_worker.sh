#!/bin/bash  
cd || exit
cd /root/dev_ws/
source install/local_setup.sh
cd src/era_5g_object_detection_distributed/
UUID=$(cat /proc/sys/kernel/random/uuid)
celery -A era_5g_object_detection_distributed.ml_service_worker worker -n $UUID
