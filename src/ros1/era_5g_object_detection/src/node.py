from multiprocessing.queues import Queue
import time
import rospy
import json

from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from queue import Queue
import cv2
#from era_5g_object_detection_common.mm_detector import MMDetector
from era_5g_object_detection_standalone.worker_mmdet import MMDetectorWorker
from era_5g_object_detection_common.mmdet_utils import MODEL_VARIANTS

image_queue = Queue(1)

class ObjectDetector(MMDetectorWorker):

    def __init__(self, image_queue: Queue, pub, **kw):
        super().__init__(image_queue, None, **kw)
        self.pub = pub
    
    def publish_results(self, results, metadata):
        detections = list()

        for result in results:
            det = dict()
            # process the results based on currently used model
            if MODEL_VARIANTS[self.model_variant]['with_masks']:
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
        r = {"timestamp": metadata["timestamp"],
            "recv_timestamp": metadata["recv_timestamp"],
            "timestamp_before_process": metadata["timestamp_before_process"],
            "timestamp_after_process": metadata["timestamp_after_process"],
            "send_timestamp": send_timestamp,
            "detections": detections}
        results = String()
        results.data = json.dumps(r)
        pub.publish(results)

pub = None

def image_callback(msg: Image):
    try:
        # Convert the ROS image message to OpenCV format
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        image_queue.put(({"timestamp": msg.header.stamp.to_nsec(), "recv_timestamp": rospy.Time.now().to_nsec()}, cv_image))
        
    except Exception as e:
        rospy.logerr(e)

def object_detector():
    global pub
    rospy.init_node('object_detector', anonymous=True)
    
    # Subscribe to the image topic
    rospy.Subscriber("/image_raw", Image, image_callback)

    # Publish to the modified image topic
    pub = rospy.Publisher("/results", String, queue_size=10)

    detector = ObjectDetector(image_queue, pub)
    detector.start()
    
    # Spin until interrupted
    while not rospy.is_shutdown():
        rospy.sleep(1)
    detector.stop()

if __name__ == '__main__':
    object_detector()