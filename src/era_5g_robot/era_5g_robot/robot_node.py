import argparse
from asyncio import tasks
from multiprocessing.sharedctypes import Value
import os.path
from argparse import ArgumentTypeError
import sys
import time
from typing import Tuple, Union
import json
import cv2  # OpenCV library.
import os
import numpy as np

if os.name != 'nt':
    from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images.

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from rclpy.task import Future
from std_msgs.msg import String
from sensor_msgs.msg import Image

from era_5g_helpers import get_path_to_assets

from era_5g_robot_interfaces.srv import StopService
from era_5g_robot_interfaces.srv import StartService

from era_5g_service_interfaces.srv import Start
from era_5g_service_interfaces.srv import Stop

from rclpy.action import ActionClient
from era_5g_action_interfaces.action import Goal5g

from threading import Event
from rclpy.callback_groups import ReentrantCallbackGroup

import ast

path_to_assets = get_path_to_assets()

IMG_PATH = os.path.join(path_to_assets, "test_image.jpg")
VIDEO_PATH = os.path.join(path_to_assets, "test_video.mp4")
USE_VIDEO = True

# Default run time (in seconds).
DEFAULT_RUN_TIME = 0


def positive_or_zero_int(string: str) -> int:
    """
    Check and convert string to positive or zero int.

    :param string: String to check.
    :return: Converted string as int.
    """
    try:
        value = int(string)
    except ValueError:
        value = -1
    if value < 0:
        raise ArgumentTypeError('value must be a positive or zero integer')
    return value


class RobotMLControlServicesClient(Node):
    """
    RobotMLControlServicesClient class, which is a subclass of the Node class.
    The node provides communication with 5G-ERA ML Control Services.
    """

    def __init__(self, service_base_name: str):
        """
        Class constructor to set up the node.

        :param service_base_name: Service base name.
        """
        super().__init__('robot_ml_control_services_client')

        # Create clients for 5G-ERA ML Control Services.
        self.start_service_client = self.create_client(Start, service_base_name + '/start')
        self.get_logger().info(service_base_name + '/start client created')
        self.stop_service_client = self.create_client(Stop, service_base_name + '/stop')
        self.get_logger().info(service_base_name + '/stop client created')

        # Wait for the services' connection.
        attempts = 0
        while attempts < 5 and not self.start_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('5G-ERA ML Control Service "Start" is not available, waiting again ...')
            attempts += 1
        if not self.start_service_client.service_is_ready():
            raise ValueError(
                '5G-ERA ML Control Service "Start" with base name "' + service_base_name + '" is not available')
        self.get_logger().info(service_base_name + '/start client connected')

        attempts = 0
        while attempts < 5 and not self.stop_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('5G-ERA ML Control Service "Stop" is not available, waiting again ...')
            attempts += 1
        if not self.stop_service_client.service_is_ready():
            raise ValueError(
                '5G-ERA ML Control Service "Stop" with base name "' + service_base_name + '" is not available')
        self.get_logger().info(service_base_name + '/stop client connected')

        self.heart_beat_subscription = self.create_subscription(String, service_base_name + "/heart_beat",
                                                                self.heart_beat_callback, 10)
        self.last_heart_beat_timestamp = time.time()

        # Future and response variables.
        self.start_request_future = None
        self.start_request_response = None
        self.stop_request_future = None
        self.stop_request_response = None

        self.get_logger().info('robot_ml_control_services_client node is running')

    def heart_beat_callback(self, msg) -> None:
        """
        Callback for heart beat.

        :param msg: Incoming message.
        """
        self.last_heart_beat_timestamp = time.time()
        self.get_logger().info('I heard heart beat: %s' % msg.data)

    def send_start_request(self) -> bool:
        """
        Send "Start" 5G-ERA ML Control Service request.

        :return: True if the start_service_client is ready and the request was sent, otherwise returns False.
        """
        if not self.services_are_alive():
            return False
        # Cancel any 5G-ERA ML Control Service unfinished requests.
        self.cancel_requests()
        # Create new Start service request.
        start_request = Start.Request()
        start_request.robot_worker_setting = ""
        # Delete old responses.
        self.delete_old_responses()
        # Send "Start" 5G-ERA ML Control Service request.
        if self.start_service_client.service_is_ready():
            self.start_request_future = self.start_service_client.call_async(start_request)
            self.start_request_future.add_done_callback(self.start_request_future_done_callback)
            return True
        else:
            return False

    def send_stop_request(self) -> bool:
        """
        Send "Stop" 5G-ERA ML Control Service request.

        :return: True if the stop_service_client is ready and the request was sent, otherwise returns False.
        """
        if not self.services_are_alive():
            return False
        # Cancel any 5G-ERA ML Control Service unfinished requests.
        self.cancel_requests()
        # Create new Stop service request.
        stop_request = Stop.Request()
        stop_request.task_id = self.start_request_response.task_id
        # Delete old responses.
        self.delete_old_responses()
        # Send "Stop" 5G-ERA ML Control Service request.
        if self.stop_service_client.service_is_ready():
            self.stop_request_future = self.stop_service_client.call_async(stop_request)
            self.stop_request_future.add_done_callback(self.stop_request_future_done_callback)
            return True
        else:
            return False

    def start_request_future_done_callback(self, future: Future) -> None:
        """
        "Start" 5G-ERA ML Control Service request Future done callback.

        :param future: Future object of request.
        """
        if future.done():
            try:
                self.start_request_response = future.result()
            except Exception as e:
                self.start_request_response = None
                self.get_logger().info(
                    '5G-ERA ML Control Service call failed %r.' % (e,))
            else:
                self.get_logger().info(
                    'Result of "Start" 5G-ERA ML Control Service request: \n '
                    'success: %r, message: "%s", task_id: %s, \n '
                    'data_topic: "%s", result_topic: "%s", worker_constraints: "%s"' %
                    (self.start_request_response.success, self.start_request_response.message,
                     self.start_request_response.task_id,
                     self.start_request_response.data_topic, self.start_request_response.result_topic,
                     self.start_request_response.worker_constraints))

    def stop_request_future_done_callback(self, future: Future) -> None:
        """
        "Stop" 5G-ERA ML Control Service request Future done callback.

        :param future: Future object of request.
        """
        if future.done():
            try:
                self.stop_request_response = future.result()
            except Exception as e:
                self.stop_request_response = None
                self.get_logger().info(
                    '5G-ERA ML Control Service call failed %r.' % (e,))
            else:
                self.get_logger().info(
                    'Result of "Stop" 5G-ERA ML Control Service request: \n '
                    'success: %r, message: "%s"' %
                    (self.stop_request_response.success, self.stop_request_response.message))

    def get_topic_names(self) -> Union[Tuple[str, str], Tuple[None, None]]:
        """
        Try to get the topic name if "Start" 5G-ERA ML Control Service has already started and responded.

        :return: Data topic name, Result topic name.
        """
        if self.start_request_response:
            return self.start_request_response.data_topic, self.start_request_response.result_topic
        else:
            return None, None

    def service_is_running(self) -> bool:
        """
        Is the 5G-ERA ML Control Service in running state?
        Call also heartbeat detection function.

        :return: True if the service is running, otherwise returns False.
        """
        if not self.services_are_alive():
            return False
        if self.start_request_response:
            return True
        else:
            return False

    def services_are_alive(self):
        """
        Are the 5G-ERA ML Control Service alive? Heartbeat functionality.
        If it detects that a heartbeat message has not arrived for more than 2 seconds or the ML Control
        Service "Start" or "Stop" is not available, it cancels any requests to the ML Control Service.
        :return:
        """
        if time.time() > self.last_heart_beat_timestamp + 2:
            self.get_logger().info(
                '5G-ERA ML Control Services did not publish to "heart beat" topic for more than 2 seconds')
            self.cancel_requests()
            self.delete_old_responses()
            return False
        else:
            if not self.stop_service_client.service_is_ready():
                self.get_logger().info('5G-ERA ML Control Service "Stop" is not available')
                self.cancel_requests()
                self.delete_old_responses()
                return False
            if not self.start_service_client.service_is_ready():
                self.get_logger().info('5G-ERA ML Control Service "Start" is not available')
                self.cancel_requests()
                self.delete_old_responses()
                return False
            return True

    def cancel_requests(self):
        """
        Cancel any 5G-ERA ML Control Service unfinished requests.
        """
        if self.stop_request_future and not self.stop_request_future.done():
            self.stop_request_future.cancel()
        if self.start_request_future and not self.start_request_future.done():
            self.start_request_future.cancel()

    def delete_old_responses(self):
        """
        Delete old responses.
        """
        self.start_request_response = None
        self.stop_request_response = None


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

        self.resourceStatus = None  # indicates the current status of the deployed service
        self.service_id = None  # GUID of the control service
        self.task_id = None  # set using the Start service
        self.css_node_name = "/ml_control_services"  # TODO: should be obtained from the MW (?)
        self.css_deployed_event = Event()  # marks that that CSS was deployed (does not have to be deployed successfully)

        self.callback_group = ReentrantCallbackGroup()  # needed to be able to process ActionServer5G feedback when start_service_callback is active
        # Create services for middleware communication.
        self.start_service = self.create_service(StartService, node_name + '/start_service',
                                                 self.start_service_callback,
                                                 callback_group=self.callback_group)
        self.get_logger().info(node_name + '/start_service is running')
        self.stop_service = self.create_service(StopService, node_name + '/stop_service', self.stop_service_callback,
                                                callback_group=self.callback_group)
        self.get_logger().info(node_name + '/stop_service is running')

        # Create the timer.
        self.timer = self.create_timer(0.2, self.image_publisher_callback)

        # Create a VideoCapture object.
        # The argument '0' gets the default webcam.
        # self.cap = cv2.VideoCapture(0)
        self.img = cv2.imread(IMG_PATH)
        # cv2.imshow("test", self.img)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()

        if USE_VIDEO:
            # Create a VideoCapture object.
            self.video_capture = cv2.VideoCapture(VIDEO_PATH)
        else:
            self.img = cv2.imread(IMG_PATH)

        # Store published images for further processing
        self.image_storage = dict()

        # Frame ID.
        self.frame_id = 0

        # Connect to the ActionServer5G
        self._action_client = ActionClient(self, Goal5g, 'goal_5g',
                                           callback_group=self.callback_group)  # instantiate a new client for the ActionServer5G
        self.get_logger().info("Waiting for ActionServer5G")
        if not self._action_client.wait_for_server(15):
            raise ValueError(
                'ActionServer5G is not available')
        self.get_logger().info("Connected to ActionServer5G")

        # Used to convert between ROS and OpenCV images.
        if os.name != 'nt':
            self.br = CvBridge()
        self.get_logger().info(node_name + ' node is running')

    def goal_response_callback(self,
                               future):  # Method to handle what to do after goal was either rejected or accepted by ActionServer5G.
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected!')
            self.css_deployed_event.set()
            return

        self.get_logger().info('Goal accepted! ')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):  # Method to handle what to do after receiving the action result
        result = future.result().result
        self.css_deployed_event.set()
        self.get_logger().info('Result: {0}'.format(result.result))

    def feedback_callback(self, feedback_msg):  # Obtaining the feedback from the ActionServer5G

        feedback = feedback_msg.feedback
        resource_status = ast.literal_eval(
            feedback.feedback_resources_status)  # get resource feedback for specific action id

        self.resourceStatus = resource_status["ActionSequence"][0]["Services"][0][
            "ServiceStatus"]  # Obtain service status for our deplyoed service

        if self.resourceStatus == "Active":
            service_id = resource_status["ActionSequence"][0]["Services"][0][
                "ServiceInstanceId"]  # obtain the ID of the service
            self.service_id = service_id.replace("-", "_")
            self.get_logger().info('Service id: ' + str(self.service_id))
            self.css_deployed_event.set()
        # self.get_logger().info('Received feedback: {0}'.format(resource_status))
        self.get_logger().info('Resource status: ' + str(self.resourceStatus))

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Canceling goal...")
        self.css_deployed_event.set()

    def send_action_server_goal(self, action_reference: int) -> None:
        """
        Creates a goal with specified action_reference and sends it to the ActionServer5G

        :param action_reference: 0 for deploying the service, -1 for removing the service
        """
        goal_msg = Goal5g.Goal()
        goal_msg.goal_taskid = self.task_id  # task id
        goal_msg.action_reference = action_reference  # Action reference
        self.get_logger().info(
            f"Connecting to ActionServer5G to send a new goal with ID {self.task_id} and ActionReference {action_reference}")
        if not self._action_client.wait_for_server(15):
            raise ValueError('ActionServer5G is not available')
        self.get_logger().info("Connected, trying to deploy service")
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def start_service_callback(self, request: StartService.Request,
                               response: StartService.Response) -> StartService.Response:
        """
        Start service callback.

        :param request: Service request with 5G-ERA ML Control Services base name.
        :param response: StartService Response.
        :return: StartService Response.
        """
        self.task_id = request.service_base_name  # TODO: rename this field in the action description
        self.css_deployed_event.clear()  # clears any previous set of the event
        self.send_action_server_goal(0)  # deploys the service
        self.get_logger().info("Waiting for service to be deployed")
        self.css_deployed_event.wait()  # waits until the service is deployed or the goal is rejected
        self.get_logger().info("Service deployed")

        if self.resourceStatus != "Active":  # if the deployment process failed, return false
            response.message = "Failed to deploy service."
            response.success = False
            return response

        if not self.robot_ml_control_services_client:
            # Try to create the RobotMLControlServicesClient node.
            try:
                self.robot_ml_control_services_client = RobotMLControlServicesClient(
                    self.css_node_name + "_" + self.service_id)
                self.executor.add_node(self.robot_ml_control_services_client)
                response.message = 'Robot Service Client successfully created and connected with 5G-ERA ML Control Services (' + self.css_node_name + self.service_id + '). '
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

    def stop_service_callback(self, request: StopService.Request,
                              response: StopService.Response) -> StopService.Response:
        """
        Stop service callback.

        :param request: Service request with 5G-ERA ML Control Services base name, Unused in this example.
        :param response: StopService Response.
        :return: StopService Response.
        """

        self.send_action_server_goal(-1)  # turn off the deployed service

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

    def image_publisher_callback(self) -> None:
        """
        Callback function.
        This function gets called every 0.1 seconds.
        """
        frame = self.img
        if os.name == 'nt':
            image_message = Image()
        else:
            image_message = self.br.cv2_to_imgmsg(self.img)

        if USE_VIDEO:
            # Capture frame-by-frame as the video frame.
            ret, frame = self.video_capture.read()
            if not ret:
                # Repeat video
                self.video_capture.set(cv2.CAP_PROP_POS_FRAMES, 0)
                ret, frame = self.video_capture.read()
            # Create image message.
            if ret:
                frame = cv2.resize(frame, (640, 360))
                if os.name == 'nt':
                    image_message = Image()
                else:
                    image_message = self.br.cv2_to_imgmsg(frame)
            else:
                self.get_logger().info('Video is not available')
                if self.robot_ml_control_services_client and self.robot_ml_control_services_client.service_is_running():
                    # Call stop service.
                    if self.robot_ml_control_services_client.send_stop_request():
                        self.get_logger().info(
                            'Robot Service Client successfully sent "Stop" request to 5G-ERA ML Control Services')
                    else:
                        self.get_logger().info('5G-ERA ML Control Service "Stop" is not available')
                self.timer.cancel()

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
                # Set frame id.
                image_message.header.frame_id = str(self.frame_id)
                image_message.header.stamp = self.get_clock().now().to_msg()
                self.frame_id += 1
                # Publish image message.
                self.publisher.publish(image_message)

                # Store image internally
                frame_key = "%s.%s" % (image_message.header.stamp.sec, image_message.header.stamp.nanosec)
                self.image_storage[frame_key] = frame

                # Display the message on the console.
                self.get_logger().info('Publishing image %s' % image_message.header.frame_id)

    def result_callback(self, msg) -> None:
        """
        Callback for received results.

        :param msg: Incoming message.
        """
        self.get_logger().info('I heard: %s' % msg.data)

        # Load JSON.
        data = json.loads(msg.data)
        header = data["header"]
        # Get stored frame.
        frame = self.image_storage[header["stamp"]]
        # Load detections.
        detections = data["detections"]

        for d in detections:
            score = d["score"]
            cls = d["class"]
            cls_name = d["class_name"]
            # Draw detection into frame.
            # x, y, w, h = d["bbox"]
            # cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
            if cls_name in ["face", "person"]:
                x1, y1, x2, y2 = [int(coord) for coord in d["bbox"]]
                cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)

        cv2.imshow("Detection_results", frame)
        cv2.waitKey(1)


def main(args=None):
    """Main function."""
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter,
                                     description='ROS2 5G-ERA robot testing app')
    parser.add_argument('--run_time', '-r', dest='run_time', type=positive_or_zero_int, action='store',
                        help='run time (0 means that it will run until it is interrupted', default=DEFAULT_RUN_TIME)
    parser.add_argument('--robot_node_name', '-n', dest='robot_node_name', action='store',
                        help='Robot logic node name', default='robot_logic')
    arguments = parser.parse_args()

    # Initialize the rclpy library.
    rclpy.init(args=args)

    # Create executor for multiple nodes.
    executor = None
    """if __debug__:
        # this is useful for debugging, because the process stops on any unhandled exception
        from rclpy.executors import SingleThreadedExecutor
        executor = SingleThreadedExecutor()  # PriorityExecutor()
    else:
        print("using multi")
        # this swallows exceptions (not sure why), which makes debugging hard..."""
    executor = MultiThreadedExecutor()  # we need to use MultiThreadedExecutor to be able to deploy service during the Start service call

    # Create the RobotLogic node.
    robot_logic = RobotLogic(arguments.robot_node_name)
    executor.add_node(robot_logic)

    try:
        if arguments.run_time == 0:
            # Keep publishing.
            executor.spin()
        else:
            # Publish messages for a set run time.
            t_end = time.time() + arguments.run_time
            while time.time() < t_end:
                executor.spin_once()
    except KeyboardInterrupt:
        pass
    except BaseException:
        print('Exception in robot:', file=sys.stderr)
        raise
    finally:
        executor.shutdown()
        rclpy.shutdown()
        pass


if __name__ == '__main__':
    main()
