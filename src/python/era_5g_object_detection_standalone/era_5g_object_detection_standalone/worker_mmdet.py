import time
from typing import Deque


from era_5g_object_detection_common.mm_detector import MMDetector
from era_5g_object_detection_standalone.worker import Worker
from era_5g_object_detection_common.mmdet_utils import MODEL_VARIANTS


class MMDetectorWorker(Worker, MMDetector):
    """
    Worker object for the universal detector based on MMDET package.
    """

    def __init__(self, image_queue: Deque, sio, **kw):
        """
        Constructor

        Args:
            image_queue (Queue): Queue with all to-be-processed images.
            app (_type_): A flask app for results publishing.
        """
        super().__init__(image_queue=image_queue, sio=sio, **kw)

    def publish_results(self, results, metadata):
        """
        Publishes the results to the robot.

        Args:
            metadata (_type_): NetApp-specific metadata related to processed image. TODO: describe the metadata
            results (_type_): The results of the detection. TODO: describe the results format
        """
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

        send_timestamp = time.time_ns()

        # add timestamp to the results
        r = {
            "timestamp": metadata["timestamp"],
            "recv_timestamp": metadata["recv_timestamp"],
            "timestamp_before_process": metadata["timestamp_before_process"],
            "timestamp_after_process": metadata["timestamp_after_process"],
            "send_timestamp": send_timestamp,
            "detections": detections,
        }
        self.sio.emit("message", r, namespace="/results", to=metadata["websocket_id"])
