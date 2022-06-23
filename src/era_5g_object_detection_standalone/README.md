# Testing commands

First terminal - ML_service
```
source install/local_setup.sh
ros2 run era_5g_object_detection_standalone ml_service
```

Second terminal - Robot
```
source install/local_setup.sh
ros2 run era_5g_robot robot_node

```

Third terminal - Robot_control
```
source install/local_setup.sh
export DEBUG_TASK_ID=00000000_0000_0000_0000_000000000000
ros2 service call robot_logic/start_service era_5g_robot_interfaces/srv/StartService "{service_base_name: /ml_control_services_$DEBUG_TASK_ID}"
```
