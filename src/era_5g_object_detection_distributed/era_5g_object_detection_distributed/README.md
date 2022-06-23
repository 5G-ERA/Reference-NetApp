# Testing commands - without Kubernetes

First terminal - ML Service Worker
```
source install/local_setup.sh
cd src/era_5g_object_detection_distributed/
celery -A era_5g_object_detection_distributed.ml_service_worker worker
```
(It might be necessary to add --pool=solo in virtual machine when testing without k8s.)

Second terminal - ML Service Interface
```
source install/local_setup.sh
ros2 run era_5g_object_detection_distributed ml_service
```

Third terminal - Robot
```
source install/local_setup.sh
ros2 run era_5g_robot robot_node

```

Fourth terminal - Robot_control
```
source install/local_setup.sh
export DEBUG_TASK_ID=00000000_0000_0000_0000_000000000000
ros2 service call robot_logic/start_service era_5g_robot_interfaces/srv/StartService "{service_base_name: /ml_control_services_$DEBUG_TASK_ID}"
```
