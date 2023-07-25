import os
import rospy
import json

from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from collections import deque
from typing import Deque

from era_5g_object_detection_standalone.worker import BATCH_SIZE
from era_5g_object_detection_standalone.worker_mmdet import MMDetectorWorker
from era_5g_object_detection_common.mmdet_utils import MODEL_VARIANTS

# limited size is used to discard outdated images if the processing takes too long, so the queue/delay does not rise indefinitely
image_queue = deque(maxlen=BATCH_SIZE + 1)

INPUT_TOPIC = os.getenv("INPUT_TOPIC", None)
OUTPUT_TOPIC = os.getenv("OUTPUT_TOPIC", None)


class ObjectDetector(MMDetectorWorker):
    def __init__(self, image_queue: Deque, pub, **kw):
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

        send_timestamp = rospy.Time.now().to_nsec()

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
        pub.publish(results)


pub = None
bridge = CvBridge()


def image_callback(msg: Image):
    try:
        # Convert the ROS image message to OpenCV format
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    except CvBridgeError as e:
        rospy.logerr(f"Can't convert image to cv2. {e}")
        return

    image_queue.appendleft(
        (
            {
                "timestamp": msg.header.stamp.to_nsec(),
                "recv_timestamp": rospy.Time.now().to_nsec(),
            },
            cv_image,
        )
    )


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
        print(
            "INPUT_TOPIC and OUTPUT_TOPIC environment variables needs to be specified!"
        )
    else:
        object_detector()
