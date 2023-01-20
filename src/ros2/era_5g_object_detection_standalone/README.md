# Testing commands
These commands are intended for local testing without running middleware. The middleware repository is required to be present, but only for imports. Specifically, the robot needs to import the module 'era_5g_action_interfaces' (make sure the middleware/install/local_setup.sh is also sourced).

Common init commands - all terminals
```
source ML_Toolboxes/install/local_setup.sh
source middleware/install/local_setup.sh
source ML_Toolboxes/set_environment.sh
export NETAPP_ID=123
```

First terminal - ML_service
```
source install/setup.sh
ros2 run era_5g_object_detection_standalone ml_service
```

Second terminal - Fake Action Server
```
ros2 run era_5g_fake_action_server FakeActionServerNode
```

Third terminal - Robot
```
ros2 run era_5g_robot robot_node
```

Fourth terminal - Robot_control
```
ros2 service call robot_logic/start_service era_5g_robot_interfaces/srv/StartService "{service_base_name: /ml_control_services_$NETAPP_ID}"
```
