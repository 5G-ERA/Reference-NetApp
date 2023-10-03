import json
import os
from queue import Queue
from typing import Optional

import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.time import Time
from sensor_msgs.msg import Image
from std_msgs.msg import String

from era_5g_interface.task_handler_internal_q import TaskHandlerInternalQ, QueueFullAction
from era_5g_object_detection_common.mmdet_utils import MODEL_VARIANTS
from era_5g_object_detection_standalone.worker import BATCH_SIZE
from era_5g_object_detection_standalone.worker_mmdet import MMDetectorWorker

# limited size is used to discard outdated images if the processing takes too long, so the queue/delay does not rise indefinitely
image_queue = Queue(maxsize=BATCH_SIZE + 1)

task_handler = TaskHandlerInternalQ("ros_node", image_queue, if_queue_full=QueueFullAction.DISCARD_OLDEST)

INPUT_TOPIC = os.getenv("INPUT_TOPIC", None)
OUTPUT_TOPIC = os.getenv("OUTPUT_TOPIC", None)

node: Optional[Node] = None

bridge = CvBridge()


class ObjectDetector(MMDetectorWorker):
    def __init__(self, image_queue: Queue, pub: Publisher, **kw):
        super().__init__(image_queue, None, **kw)
        self.pub = pub

    def publish_results(self, results, metadata):
        detections = list()

        for result in results:
            det = dict()
            # process the results based on currently used model
            if MODEL_VARIANTS[self.model_variant]["with_masks"]:
                bbox, score, cls_id, cls_name, mask = result
                det["mask"] = mask
            else:
                bbox, score, cls_id, cls_name = result
            det["bbox"] = [float(i) for i in bbox]
            det["score"] = float(score)
            det["class"] = int(cls_id)
            det["class_name"] = str(cls_name)

            detections.append(det)

        send_timestamp = node.get_clock().now().nanoseconds

        # add timestamp to the results
        r = {
            "timestamp": metadata["timestamp"],
            "recv_timestamp": metadata["recv_timestamp"],
            "timestamp_before_process": metadata["timestamp_before_process"],
            "timestamp_after_process": metadata["timestamp_after_process"],
            "send_timestamp": send_timestamp,
            "detections": detections,
        }
        results = String()
        results.data = json.dumps(r)
        self.pub.publish(results)


def image_callback(msg: Image):
    try:
        # Convert the ROS image message to OpenCV format
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    except CvBridgeError as e:
        node.get_logger().error(f"Can't convert image to cv2. {e}")
        return
    if cv_image is not None:
        metadata = {"timestamp": Time.from_msg(msg.header.stamp).nanoseconds,
                    "recv_timestamp": node.get_clock().now().nanoseconds}
        task_handler.store_data(metadata, cv_image)
    else:
        node.get_logger().warning("Empty image received!")


def object_detector():
    global node
    rclpy.init()
    node = rclpy.create_node("object_detector")
    detector1 = None

    try:
        # Subscribe to the image topic
        node.create_subscription(Image, INPUT_TOPIC, image_callback, 1)

        # Publish to the modified image topic
        pub = node.create_publisher(String, OUTPUT_TOPIC, 10)

        detector1 = ObjectDetector(image_queue, pub)
        detector1.start()

        # Spin until interrupted
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=1.0)
        detector1.stop()
    except KeyboardInterrupt:
        if detector1 is not None:
            detector1.stop()
        exit()


if __name__ == "__main__":
    if None in [INPUT_TOPIC, OUTPUT_TOPIC]:
        print(
            "INPUT_TOPIC and OUTPUT_TOPIC environment variables needs to be specified!"
        )
    else:
        object_detector()
