import logging
import time
from abc import ABC
from queue import Queue, Empty
from typing import Dict, List, Callable, Any

from era_5g_object_detection_common.image_detector import ImageDetector

BATCH_SIZE = 5

logger = logging.getLogger(__name__)


class Worker(ImageDetector, ABC):
    """Worker object for image processing in standalone variant. Reads data from passed queue, performs detection and
    returns results using the flask app. Needs to be inherited to implement the process_image method.
    """

    def __init__(self, image_queue: Queue, send_function: Callable[[Dict], None], **kw) -> None:
        """Constructor.

        Args:
            image_queue (Queue): The queue with all to-be-processed images
            send_function (Callable[[Dict], None]): Callback used to send results.
        """

        super().__init__(**kw)
        self.image_queue = image_queue
        self.send_function = send_function

    def run(self) -> None:
        """Periodically reads images from python internal queue process them."""

        logger.info(f"{self.name} thread is running.")

        while not self.stop_event.is_set():
            images = []
            metadata: List[Dict] = []

            # Try to get a batch (or at least something) from the queue.
            for _ in range(BATCH_SIZE):
                try:
                    data = self.image_queue.get(block=True, timeout=0.1)
                    mt, image = data
                except Empty:
                    break
                metadata.append(mt)
                images.append(image)

            assert len(images) == len(metadata)

            if not images:
                logger.debug("Not enough data!")
                continue

            logger.debug(f"Batch size: {len(images)}")

            ts_before = time.perf_counter_ns()
            for idx in range(len(metadata)):
                metadata[idx]["timestamp_before_process"] = ts_before

            detections = self.process_images(images)

            ts_after = time.perf_counter_ns()
            for mt in metadata:
                mt["timestamp_after_process"] = ts_after

            logger.debug(f"Processing took {(ts_after - ts_before) / 10 ** 9}")

            assert len(detections) == len(images)

            for idx in range(len(detections)):
                self.publish_results(detections[idx], metadata[idx])

    def publish_results(self, results: Dict, metadata: Dict[str, Any]) -> None:
        """Publishes the results to the robot.

        Args:
            results (Dict): The results of the detection. TODO: Result format detail.
            metadata (Dict[str, Any]): 5G-ERA Network Application specific metadata related to processed image.
                TODO: describe the metadata
        """

        detections = list()
        for bbox, score, cls_id, cls_name in results:
            det = dict()
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
