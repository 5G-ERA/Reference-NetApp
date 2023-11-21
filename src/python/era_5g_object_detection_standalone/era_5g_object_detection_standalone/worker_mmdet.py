import time
from queue import Queue
from typing import Callable, Dict, Any

from era_5g_object_detection_common.mm_detector import MMDetector
from era_5g_object_detection_common.mmdet_utils import MODEL_VARIANTS
from era_5g_object_detection_standalone.worker import Worker


class MMDetectorWorker(Worker, MMDetector):
    """Worker object for the universal detector based on MMDET package."""

    def __init__(self, image_queue: Queue, send_function: Callable[[Dict], None], **kw):
        """Constructor.

        Args:
            image_queue (Queue): Queue with all to-be-processed images.
            send_function (Callable[[Dict], None]): Callback used to send results.
        """

        super().__init__(image_queue=image_queue, send_function=send_function, **kw)

    def publish_results(self, results: Dict, metadata: Dict[str, Any]):
        """Publishes the results to the robot.

        Args:
            results (Dict): The results of the detection. TODO: Result format detail.
            metadata (Dict[str, Any]): 5G-ERA Network Application specific metadata related to processed image.
                TODO: describe the metadata
        """

        detections = list()
        for result in results:
            det = dict()
            # Process the results based on currently used model.
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

        send_timestamp = time.perf_counter_ns()

        self.latency_measurements.store_latency(send_timestamp - metadata["recv_timestamp"])

        # Add timestamp to the results.
        r = {
            "timestamp": metadata.get("timestamp", 0),
            "recv_timestamp": metadata.get("recv_timestamp", 0),
            "timestamp_before_process": metadata["timestamp_before_process"],
            "timestamp_after_process": metadata["timestamp_after_process"],
            "send_timestamp": send_timestamp,
            "detections": detections,
        }
        self.send_function(r)
