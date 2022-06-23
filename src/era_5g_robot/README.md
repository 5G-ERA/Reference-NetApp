# Test robot_logic - robot_ml_control_services_client - ml_control_services

Instalation (ROS2 Galactic), building
```console
cd Reference-NetApp
colcon build
```
First terminal (same dir), fisrt robot (robot_logic):
```console
source install/setup.sh
ros2 run era_5g_robot robot_node
```
Second terminal (same dir), second robot (robot_logic_2):
```console
source install/setup.sh
ros2 run era_5g_robot robot_node -n robot_logic_2
```
Third terminal (same dir), era_5g_action_server:
```console
source install/setup.sh
ros2 run era_5g_action_server ActionServerNode
```
Fourth terminal (same dir), commands to robots:
```console
source install/setup.sh
```
External command to the first robot (robot_logic - start_service) that the robot_ml_control_services_client send a ml_control_services request with the base name "ml_control_services" with a request to assign topic names and start the service (ml_service_start).
```console
ros2 service call robot_logic/start_service era_5g_robot_interfaces/srv/StartService "{service_base_name: GUID_OF_THE_TASK}"
```
External command to the second robot (robot_logic_2 - start_service) that the robot_ml_control_services_client send a ml_control_services request with the base name "ml_control_services" with a request to assign topic names and start the service (ml_service_start).
```console
ros2 service call robot_logic_2/start_service era_5g_robot_interfaces/srv/StartService "{service_base_name: GUID_OF_THE_TASK}"
```
External command to the first robot (robot_logic - stop_service) that robot_ml_control_services_client send a ml_control_services request to stop the service (ml_service_stop).
```console
ros2 service call robot_logic/stop_service era_5g_robot_interfaces/srv/StopService
```
External command to the second robot (robot_logic_2 - stop_service) that robot_ml_control_services_client send a ml_control_services request to stop the service (ml_service_stop).
```console
ros2 service call robot_logic_2/stop_service era_5g_robot_interfaces/srv/StopService
```

## Step-by-step Integration of era_5g_action_client

The reference era_5g_action_client is available in this [repository](https://github.com/5G-ERA/middleware-actionserver). Reference integration is available in this package. 

1. Import ActionClient and Goal5G

```python 
from rclpy.action import ActionClient
from era_5g_action_interfaces.action import Goal5g
```

2. Create an instance of ActionClient and check if it is available:

```python
self._action_client  = ActionClient(self, Goal5g, 'goal_5g', callback_group=self.callback_group)  # instantiate a new client for the era_5g_action_client
if not self._action_client.wait_for_server(15):
    raise ValueError(
        'ActionServer5G is not available')
```

3. Ask middleware to provide a specific NetApp by sending a goal to the era_5g_action_server

```python
goal_msg = Goal5g.Goal()
goal_msg.goal_taskid = self.task_id #task id
goal_msg.action_reference = 0 # Action reference
self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
self._send_goal_future.add_done_callback(self.goal_response_callback)
```

4. Check if the goal was accepted or rejected by the era_5g_action_server in the goal_response_callback

```python 
def goal_response_callback(self, future): # Method to handle what to do after goal was either rejected or accepted by ActionServer5G.
    goal_handle = future.result()
    if not goal_handle.accepted:
        self.get_logger().info('Goal rejected!')
        return
    self.get_logger().info('Goal accepted! ')
    self._get_result_future = goal_handle.get_result_async()
    self._get_result_future.add_done_callback(self.get_result_callback)
```

5. Create the get_result_callback, which is called after the work is done

```python
 def get_result_callback(self, future): # Method to handle what to do after receiving the action result
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.result))
```

6. Create feedback_callback, which is periodicaly called with the current state of the NetApp and it should be used to check if the NetApp is ready and what is the ServiceInstanceId of the NetApp

```python
def feedback_callback(self, feedback_msg): # Obtaining the feedback from the ActionServer5G        
    feedback = feedback_msg.feedback
    resource_status = ast.literal_eval(feedback.feedback_resources_status) # get resource feedback for specific action id
    self.resourceStatus = resource_status["ActionSequence"][0]["Services"][0]["ServiceStatus"] # Obtain service status for our deployed service
    
    if self.resourceStatus == "Active": # If the NetApp is ready
        service_id = resource_status["ActionSequence"][0]["Services"][0]["ServiceInstanceId"] # obtain the ID of the service
        self.service_id = service_id.replace("-", "_") # The css needs the GUID to contains underscores instead of dashes
        self.get_logger().info('Service id: ' + str(self.service_id))
```

7. To stop the NetApp, simply use the send goal method from step 3, with the Action reference equal to -1

## Step-by-step Integration of robot_ml_control_services_client

1. TODO

## Step-by-step Integration of heartbeat functionality

1. TODO