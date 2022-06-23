# Service implementation using distributed Celery workers

import json
import logging
import os
import uuid
import cv2
from time import sleep
from concurrent.futures import ThreadPoolExecutor
from queue import Empty, Full, Queue
from typing import Dict  # for Python 3.9, just 'dict' will be fine

import numpy as np
import rclpy  # Python library for ROS 2
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
from rclpy.executors import Executor
from rclpy.node import Node  # Handles the creation of nodes
from era_5g_helpers import get_path_to_assets
from era_5g_object_detection_distributed.common import ThreadBase, get_logger as get_thread_logger
from era_5g_service_interfaces.srv import Start, StateGet, StateReset, StateSet, Stop
from sensor_msgs.msg import Image  # Image is the message type
from std_msgs.msg import String
import sys
import era_5g_helpers

from era_5g_object_detection_distributed.ml_service_worker import detector_task

path_to_assets = get_path_to_assets()

if os.name == 'nt':
    IMG_PATH = "assets/test_image.jpg"


class PriorityExecutor(Executor):
    """
    Execute high priority callbacks in multiple threads, all others in a single thread.
    This is an example of a custom exectuor in python. Executors are responsible for managing
    how callbacks get mapped to threads. Rclpy provides two executors: one which runs all callbacks
    in the main thread, and another which runs callbacks in a pool of threads. A custom executor
    should be written if neither are appropriate for your application.
    """

    def __init__(self):
        super().__init__()
        self.high_priority_nodes = set()
        self.hp_executor = ThreadPoolExecutor(max_workers=os.cpu_count() or 4)
        self.lp_executor = ThreadPoolExecutor(max_workers=1)

    def add_high_priority_node(self, node):
        self.high_priority_nodes.add(node)
        # add_node inherited
        self.add_node(node)

    def spin_once(self, timeout_sec=None):
        """
        Execute a single callback, then return.
        This is the only function which must be overridden by a custom executor. Its job is to
        start executing one callback, then return. It uses the method `wait_for_ready_callbacks`
        to get work to execute.
        :param timeout_sec: Seconds to wait. Block forever if None. Don't wait if <= 0
        :type timeout_sec: float or None
        """
        # wait_for_ready_callbacks yields callbacks that are ready to be executed
        try:
            handler, group, node = self.wait_for_ready_callbacks(timeout_sec=timeout_sec)
        except StopIteration:
            pass
        else:
            if node in self.high_priority_nodes:
                self.hp_executor.submit(handler)
            else:
                self.lp_executor.submit(handler)


class ResultsReader(ThreadBase):

    def __init__(self, logger, name, jobs_info_que, result_publisher):
        super().__init__(logger, name)

        self.stopped = False
        self.time = None
        self.fps = 0.0
        self.logger = logger
        self._jobs_info_que = jobs_info_que
        self._result_publisher = result_publisher
        
        self.jobs_in_process = []

    def _run(self):

        self.logger.info(f"Results reader thread for {self.name} is running.")

        while not self.stopped:
            # Check for newly created jobs
            try:
                job = self._jobs_info_que.get_nowait()
                self.jobs_in_process.append(job)
            except Empty:
                pass
            
            # Check for completed jobs
            jobs_to_remove = set()
            for job in self.jobs_in_process:
                
                if job.state == "SUCCESS":
                    jobs_to_remove.add(job)
                    result = job.get()
                    self.publish_results(result)
                    
                elif job.state == "REVOKED":
                    jobs_to_remove.add(job)

                elif job.state == "FAILURE":
                    jobs_to_remove.add(job)
                    self.logger.info(f"Task {job.task_id} failed.")
                    # TODO: optional error handling
                        
            # Remove completed jobs
            if len(jobs_to_remove):
                self.jobs_in_process = [job for job in self.jobs_in_process if job not in jobs_to_remove]
            else: 
                sleep(0.02)
        
    def publish_results(self, data):
        metadata, raw_results = data

        # TODO this should definitely be a ROS message, not dict
        results = dict()
        results["header"] = dict()
        stamp = metadata["header"].stamp
        results["header"]["stamp"] = "%d.%d" % (stamp.sec, stamp.nanosec)
        results["header"]["frame_id"] = metadata["header"].frame_id

        results["detections"] = []

        for (bbox, score, cls_id, cls_name) in raw_results:
            det = dict()
            det["bbox"] = [float(i) for i in bbox]
            det["score"] = float(score)
            det["class"] = int(cls_id)
            det["class_name"] = str(cls_name)
            # TODO: masks

            results["detections"].append(det)
        
        msg = String()
        msg.data = json.dumps(results)
        self._result_publisher.publish(msg)
        self.logger.info(f"ResultsReader: {self.name} - Publishing results: {results}")


class TaskHandler(Node):
    """
    Create an TaskHandler class, which is a subclass of the Node class.
    """

    def __init__(self, task_id: str):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__(task_id)

        self._jobs_info_que = Queue()

        # Create results publisher
        self._result_publisher = self.create_publisher(String, f"/tasks/{self.get_name()}/results", 10)

        # Create the subscriber. This subscriber will receive an Image
        # from the video_frames topic. The queue size is 10 messages.
        self._subscription = self.create_subscription(
            Image,
            f"/tasks/{self.get_name()}/data",
            self.image_listener_callback,
            100)
        self._subscription  # prevent unused variable warning

        # Used to convert between ROS and OpenCV images
        self._br = CvBridge()

        # Create thread for reading results
        logger = get_thread_logger(log_level=logging.INFO)
        self.results_reader = ResultsReader(logger, task_id, self._jobs_info_que, self._result_publisher)
        self.results_reader.start(daemon=True)

        self.get_logger().info(f"TaskHandler node: {self.get_name()} created")

    def image_listener_callback(self, data: Image):
        """
        Image callback function.
        """
        # Convert ROS Image message to OpenCV image
        if os.name == 'nt':
            current_frame = cv2.imread(IMG_PATH)
        else:
            current_frame = self._br.imgmsg_to_cv2(data)

        # Create metadata for image
        metadata = {"node_name": self.get_name(), "header": data.header}

        job_data = (metadata, current_frame)
        
        # Start asynchronous Celery task
        job = detector_task.delay(job_data)
        
        # Pass info about the task to the thread for reading results
        try:
            self._jobs_info_que.put(job, block=False)
        except Full:
            self.get_logger().warning(f"Internal queue full, skipping data (frame_id {data.header.frame_id})...")
            job.revoke()
            return

        self.get_logger().info('Receiving image - frame_id: ' + data.header.frame_id)


TaskNodesDict = Dict[str, TaskHandler]


class ControlService(Node):

    def __init__(self, executor: Executor, robot_nodes: TaskNodesDict, image_queue) -> None:
        super().__init__('ml_control_services_' + era_5g_helpers.NETAPP_ID_ROS)

        self._executor = executor
        self._robot_nodes = robot_nodes
        #self._image_queue = image_queue

        self.start_srv = self.create_service(Start, f"~/{self.start.__name__}", self.start)
        self.stop_srv = self.create_service(Stop, f"~/{self.stop.__name__}", self.stop)
        self.state_set_srv = self.create_service(StateSet, "~/state/set", self.state_set)
        self.state_get_srv = self.create_service(StateGet, "~/state/get", self.state_get)
        self.state_reset_srv = self.create_service(StateReset, "~/state/reset", self.state_reset)
        self.heart_beat_publisher = self.create_publisher(String, "~/heart_beat", 10)
        self.heart_beat_timer = self.create_timer(1, self.publish_heart_beat)
        if era_5g_helpers.MIDDLEWARE_ADDRESS:
            self.middleware_heart_beat_timer = self.create_timer(era_5g_helpers.MIDDLEWARE_REPORT_INTERVAL,
                                                                 self.send_middleware_heart_beat)
        self.heart_beat_string = String()

        self.get_logger().info("Control Service running...")

    def send_middleware_heart_beat(self):
        # TODO compute limits
        data = {"Id": era_5g_helpers.NETAPP_ID, "HardLimit": 10, "OptimalLimit": 5,
                "CurrentRobotsCount": len(self._robot_nodes)}
        headers = {"Content-type": "application/json"}
        era_5g_helpers.send_middleware_heart_beat_request(self, headers=headers, json=data)

    def publish_heart_beat(self):
        self.heart_beat_string.data = "heart_beat"
        self.heart_beat_publisher.publish(self.heart_beat_string)
        self.get_logger().info(f"{self.get_name()} is publishing heart_beat: {self.heart_beat_string.data}")

    def start(self, request: Start.Request, response: Start.Response) -> Start.Response:

        task_id = f"t{uuid.uuid4().hex}"  # prefixed with "t" (task), as node name must not start with number
        th = TaskHandler(task_id)
        self._robot_nodes[task_id] = th
        self._executor.add_node(th)

        self.get_logger().info(f"Starting task_id: {task_id}, worker settings: {request.robot_worker_setting}.")
        response.success = True
        response.task_id = task_id

        # TODO maybe that returning topic names is redundant?
        response.data_topic = f"/tasks/{task_id}/data"
        response.result_topic = f"/tasks/{task_id}/results"

        return response

    def stop(self, request: Stop.Request, response: Stop.Response) -> Stop.Response:

        try:
            th = self._robot_nodes[request.task_id]
        except KeyError:
            response.success = False
            response.message = "Unknown task_id."
            return response

        th.destroy_node()
        self._executor.remove_node(th)

        self.get_logger().info(f"Stopping task_id: {request.task_id}.")

        response.success = True

        return response

    def state_set(self, request: StateSet.Request, response: StateSet.Response) -> StateSet.Response:
        response.success = False
        response.message = "Not implemented"
        return response

    def state_get(self, request: StateGet.Request, response: StateGet.Response) -> StateGet.Response:
        response.success = False
        response.message = "Not implemented"
        return response

    def state_reset(self, request: StateReset.Request, response: StateReset.Response) -> StateReset.Response:
        response.success = False
        response.message = "Not implemented"
        return response


def main(args=None):
    task_nodes: TaskNodesDict = dict()  # there is a node for each created task
    image_queue = Queue(1024)  # buffer of images for the detector

    # Initialize the rclpy library
    rclpy.init(args=args)

    logger = get_thread_logger(log_level=logging.INFO)

    if __debug__:
        # this is useful for debugging, because the process stops on any unhandled exception
        from rclpy.executors import SingleThreadedExecutor
        ros2_executor = SingleThreadedExecutor()  # PriorityExecutor()
    else:
        # this swallows exceptions (not sure why), which makes debugging hard...
        ros2_executor = PriorityExecutor()  # TODO maybe use MultiThreadedExecutor instead?

    ros2_executor.add_node(ControlService(ros2_executor, task_nodes, image_queue))

    try:
        ros2_executor.spin()
    except KeyboardInterrupt:
        pass
    except BaseException:
        print('Exception in 5G-ERA ML Control Service:', file=sys.stderr)
        raise
    finally:
        ros2_executor.shutdown()
        for node in ros2_executor.get_nodes():
            node.destroy_node()


if __name__ == '__main__':
    main()
