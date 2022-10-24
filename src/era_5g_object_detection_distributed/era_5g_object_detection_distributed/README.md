# Testing commands - without Kubernetes
These commands are intended for local testing without running middleware. The middleware repository is required to be present, but only for imports. Specifically, the robot needs to import the module 'era_5g_action_interfaces' (make sure the middleware/install/local_setup.sh is also sourced).

Common init commands - all terminals
```
source ML_Toolboxes/install/local_setup.sh
source middleware/install/local_setup.sh
source ML_Toolboxes/set_environment.sh
export NETAPP_ID=123
```


First terminal - ML Service Worker
```
source install/setup.sh
cd src/era_5g_object_detection_distributed/
celery -A era_5g_object_detection_distributed.ml_service_worker worker
```

Second terminal - ML Service Interface
```
source install/setup.sh
ros2 run era_5g_object_detection_distributed ml_service
```
Third terminal - Fake Action Server (optional, if we don't want to use an actual Action Server)
```
ros2 run era_5g_fake_action_server FakeActionServerNode
```

Fourth terminal - Robot
```
source install/setup.sh
ros2 run era_5g_robot robot_node

```

Fifth terminal - Robot_control
```
source install/local_setup.sh
export DEBUG_TASK_ID=00000000_0000_0000_0000_000000000000
ros2 service call robot_logic/start_service era_5g_robot_interfaces/srv/StartService "{service_base_name: /ml_control_services_$NETAPP_ID}"
```
