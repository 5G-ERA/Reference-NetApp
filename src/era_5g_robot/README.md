# Test robot_logic - robot_ml_control_services_client - ml_control_services

Following guide assumes that middleware is successfully installed and running (for more information about middleware deployment see this [repository ](https://github.com/5G-ERA/middleware)). Besides, the [era_5g_action_server](https://github.com/5G-ERA/middleware-actionserver) has to be installed as well.

Installation (ROS2 Galactic), building
```console
cd Reference-NetApp
colcon build
```
First terminal (same dir), robot_logic:
```console
source install/setup.sh
ros2 run era_5g_robot robot_node
```
Second terminal (same dir), era_5g_action_server:
```console
source install/setup.sh
ros2 run era_5g_action_server ActionServerNode
```
Third terminal (same dir), commands to robots:
```console
source install/setup.sh
```
External command to the robot (robot_logic - start_service) that the robot_ml_control_services_client send a ml_control_services request with the base name "GUID_OF_THE_TASK" with a request to assign topic names and start the service (ml_service_start).
```console
ros2 service call robot_logic/start_service era_5g_robot_interfaces/srv/StartService "{service_base_name: GUID_OF_THE_TASK}"
```

External command to the robot (robot_logic - stop_service) that robot_ml_control_services_client send a ml_control_services request to stop the service (ml_service_stop).
```console
ros2 service call robot_logic/stop_service era_5g_robot_interfaces/srv/StopService
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
        'ActionServerNode is not available')
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
def goal_response_callback(self, future): # Method to handle what to do after goal was either rejected or accepted by ActionServerNode.
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
def feedback_callback(self, feedback_msg): # Obtaining the feedback from the ActionServerNode
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

The reference implementation/integration of ``robot_ml_control_services_client`` (``RobotMLControlServicesClient`` class) is 
available in this ([repository](https://github.com/5G-ERA/Reference-NetApp/blob/master/src/era_5g_robot/era_5g_robot/robot_node.py)). 
The implementation of the RobotMLControlServicesClient class is in the same source code as the RobotLogic class, which 
is used to demonstrate robot logic, but these classes can be in separate packages.

1. Let's have these 3 variables in the ``RobotLogic`` node:

```python
class RobotLogic(Node):
    """
    RobotLogic class, which is a subclass of the Node class.
    The node represents the logic of the robot.
    """

    def __init__(self, node_name: str = 'robot_logic'):
        """
        Class constructor to set up the node.
        """
        super().__init__(node_name)
        self.robot_ml_control_services_client = None # An instance of the RobotMLControlServicesClient class
        self.publisher = None # data_topic from the ML Control Service
        self.subscription = None # result_topic from the ML Control Service
```

2. If the ``RobotLogic`` somewhere in its code wants to connect to ML Control Services using 
the ``RobotMLControlServicesClient`` class and start it, it can proceed as follows:

```python
if not self.robot_ml_control_services_client:
    # Try to create the RobotMLControlServicesClient node.
    try:
        # This should be obtained from the Middleware
        ml_service_base_name_and_guid_of_the_task = 'ML_SERVICE_BASE_NAME'+ "_" +'GUID_OF_THE_TASK'
        self.robot_ml_control_services_client = RobotMLControlServicesClient(ml_service_base_name_and_guid_of_the_task)
        self.executor.add_node(self.robot_ml_control_services_client)
        response.message = 'Robot Service Client successfully created and connected with 5G-ERA ML Control Services (' + ml_service_base_name_and_guid_of_the_task + '). '
    except ValueError as e:
        response.success = False
        response.message = str(e)
        return response

if self.robot_ml_control_services_client and not self.robot_ml_control_services_client.service_is_running():
    # Call start service.
    if self.robot_ml_control_services_client.send_start_request():
        response.success = True
        response.message += 'Robot Service Client successfully sent "Start" request to 5G-ERA ML Control Services'
    else:
        response.success = False
        response.message += '5G-ERA ML Control Service "Start" is not available'
else:
    response.success = False
    response.message = '5G-ERA ML Control Services are already in a running state'
return response
```

3. To stop the ML Service, it can proceed, for example, as follows:

```python
if not self.robot_ml_control_services_client:
    response.success = False
    response.message = 'Robot Service Client has not been created yet'
    return response

if self.robot_ml_control_services_client and self.robot_ml_control_services_client.service_is_running():
    # Call stop service.
    if self.robot_ml_control_services_client.send_stop_request():
        response.success = True
        response.message = 'Robot Service Client successfully sent "Stop" request to 5G-ERA ML Control Services'
    else:
        response.success = False
        response.message = '5G-ERA ML Control Service "Stop" is not available'
else:
    response.success = False
    response.message = '5G-ERA ML Control Services is not in running state'
return response
```

4. If an instance of ``MLControlServicesClient`` class is already created and the ML Service is running, 
the robot can, for example, send images and receive the results in this way:

```python
if not self.publisher and not self.subscription:
    # Try to get created topic name.
    if self.robot_ml_control_services_client:
        data_topic, result_topic = self.robot_ml_control_services_client.get_topic_names()
        if data_topic and result_topic:
            # Create image publisher with a given topic name.
            self.subscription = self.create_subscription(String, result_topic, self.result_callback, 10)
            self.publisher = self.create_publisher(Image, data_topic, 100)
            self.get_logger().info('Publisher on topic "%s" and subscription on topic "%s" created' % (
                data_topic, result_topic))
else:
    if self.robot_ml_control_services_client and not self.robot_ml_control_services_client.service_is_running():
        self.get_logger().info('Publisher on topic "%s" and subscription on topic "%s" destroyed' % (
            self.publisher.topic_name, self.subscription.topic_name))
        self.destroy_subscription(self.subscription)
        self.subscription = None
        self.destroy_publisher(self.publisher)
        self.publisher = None
    else:
        # Publish an image message.
        self.publisher.publish(image_message)

        # Display the message on the console.
        self.get_logger().info('Publishing image %s' % image_message.header.frame_id)
```
The heartbeat functionality is provided by ``services_are_alive`` and ``service_is_running`` functions 
of ``MLControlServicesClient`` class. If the function ``services_are_alive`` detects that a heartbeat message 
from ML Control Service has not arrived for more than 2 seconds or the ML Control Service "Start" or "Stop" 
is not available, it cancels any requests to the ML Control Service. The ``service_is_running`` function first 
checks the heartbeat (calls ``services_are_alive`` function), and then returns True if the ML Service is in running state.
