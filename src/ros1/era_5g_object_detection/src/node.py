import json
import os
from queue import Queue

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String

from era_5g_interface.task_handler_internal_q import TaskHandlerInternalQ, QueueFullAction
from era_5g_object_detection_standalone.worker import BATCH_SIZE
from era_5g_object_detection_standalone.worker_mmdet import MMDetectorWorker

# limited size is used to discard outdated images if the processing takes too long, so the queue/delay does not rise
# indefinitely
image_queue = Queue(maxsize=BATCH_SIZE + 1)

task_handler = TaskHandlerInternalQ(image_queue, if_queue_full=QueueFullAction.DISCARD_OLDEST)

INPUT_TOPIC = os.getenv("INPUT_TOPIC", None)
OUTPUT_TOPIC = os.getenv("OUTPUT_TOPIC", None)


class ObjectDetector(MMDetectorWorker):
    def __init__(self, image_queue: Queue, pub: rospy.Publisher, **kw):
        super().__init__(image_queue, self.publish, **kw)
        self.pub = pub

    def publish(self, results):
        results_msg = String()
        results_msg.data = json.dumps(results)
        self.pub.publish(results_msg)


pub = None
bridge = CvBridge()


def image_callback(msg: Image):
    try:
        # Convert the ROS image message to OpenCV format
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    except CvBridgeError as e:
        rospy.logerr(f"Can't convert image to cv2. {e}")
        return

    if cv_image is not None:
        metadata = {
            "timestamp": msg.header.stamp.to_nsec(),
            "recv_timestamp": rospy.Time.now().to_nsec(),
        }
        task_handler.store_data(metadata, cv_image)
    else:
        rospy.logwarn("Empty image received!")


def object_detector():
    global pub
    rospy.init_node("object_detector", anonymous=True)

    # Subscribe to the image topic
    rospy.Subscriber(INPUT_TOPIC, Image, image_callback, queue_size=1)

    # Publish to the modified image topic
    pub = rospy.Publisher(OUTPUT_TOPIC, String, queue_size=10)

    detector1 = ObjectDetector(image_queue, pub)
    detector1.start()

    # Spin until interrupted
    while not rospy.is_shutdown():
        rospy.sleep(1)
    detector1.stop()


if __name__ == "__main__":
    if None in [INPUT_TOPIC, OUTPUT_TOPIC]:
        print("INPUT_TOPIC and OUTPUT_TOPIC environment variables needs to be specified!")
    else:
        object_detector()
