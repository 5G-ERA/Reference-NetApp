from abc import ABC
from multiprocessing import Queue
from queue import Empty
import cv2
import time
import logging

from era_5g_object_detection_common.image_detector import ImageDetector

logger = logging.getLogger(__name__)


class Worker(ImageDetector, ABC):
    """
    Worker object for image processing in standalone variant. Reads 
    data from passed queue, performs detection and returns results using
    the flask app. Needs to be inherited to implement the process_image
    method.
    """

    def __init__(self, image_queue: Queue, sio, **kw):
        """
        Constructor

        Args:
            image_queue (Queue): The queue with all to-be-processed images
            app (_type_): The flask app for results publishing
        """

        super().__init__(**kw)
        self.image_queue = image_queue
        self.sio = sio
        self.frame_id = 0

    def run(self):
        """
        Periodically reads images from python internal queue process them.
        """

        logger.info(f"{self.name} thread is running.")

        while not self.stop_event.is_set():
            # Get image and metadata from input queue
            try:
                metadata, image = self.image_queue.get(block=True, timeout=1)
            except Empty:
                continue
            metadata["timestamp_before_process"] = time.perf_counter_ns()
            self.frame_id += 1
            # logger.info(f"Worker received frame id: {self.frame_id} {metadata['timestamp']}")
            try:
                detections = self.process_image(image)
                metadata["timestamp_after_process"] = time.perf_counter_ns()
                self.publish_results(detections, metadata)
            except Exception as e:
                logger.error(f"Exception with image processing: {e}")

        logger.info(f"{self.name} thread is stopping.")

    def publish_results(self, results, metadata):
        """
        Publishes the results to the robot

        Args:
            results (_type_): The results of the detection.
            metadata (_type_): NetApp-specific metadata related to processed image.
        """
        detections = list()
        if results is not None:
            for (bbox, score, cls_id, cls_name) in results:
                det = dict()
                det["bbox"] = [float(i) for i in bbox]
                det["score"] = float(score)
                det["class"] = int(cls_id)
                det["class_name"] = str(cls_name)

                detections.append(det)

            send_timestamp = time.perf_counter_ns()

            self.latency_measurements.store_latency(send_timestamp - metadata["recv_timestamp"])

            # TODO:check timestamp exists
            result = {"timestamp": metadata["timestamp"],
                      "recv_timestamp": metadata["recv_timestamp"],
                      "timestamp_before_process": metadata["timestamp_before_process"],
                      "timestamp_after_process": metadata["timestamp_after_process"],
                      "send_timestamp": send_timestamp,
                      "detections": detections}
            self.sio.emit('message', result, namespace="/results", to=metadata["websocket_id"])
