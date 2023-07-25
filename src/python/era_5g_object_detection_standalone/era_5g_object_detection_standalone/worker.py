from abc import ABC
import logging
import cv2
import time
from typing import Dict, List, Deque

from era_5g_object_detection_common.image_detector import ImageDetector

BATCH_SIZE = 5


class Worker(ImageDetector, ABC):
    """
    Worker object for image processing in standalone variant. Reads
    data from passed queue, performs detection and returns results using
    the flask app. Needs to be inherited to implement the process_image
    method.
    """

    def __init__(self, image_queue: Deque, sio, **kw):
        """
        Constructor

        Args:
            image_queue (Queue): The queue with all to-be-processed images
            app (_type_): The flask app for results publishing
        """

        super().__init__(**kw)
        self.image_queue = image_queue
        self.sio = sio

    def run(self):
        """
        Periodically reads images from python internal queue process them.
        """

        logging.info(f"{self.name} thread is running.")

        while not self.stop_event.is_set():
            images = []
            metadata: List[Dict] = []

            # try to get a batch (or at least something) from the queue
            for _ in range(BATCH_SIZE):
                try:
                    mt, image = self.image_queue.pop()
                except IndexError:
                    break
                metadata.append(mt)
                images.append(image)

            assert len(images) == len(metadata)

            if not images:
                logging.debug("Not enough data!")
                # TODO sleeping could be avoided with customized deque - signalling there is something to pop out using event
                time.sleep(0.1)
                continue

            logging.debug(f"Batch size: {len(images)}")

            ts_before = time.time_ns()
            for idx in range(len(metadata)):
                metadata[idx]["timestamp_before_process"] = ts_before

                if not metadata[idx].get("decoded", True):
                    images[idx] = cv2.imdecode(image, cv2.IMREAD_COLOR)

            detections = self.process_images(images)

            ts_after = time.time_ns()
            for mt in metadata:
                mt["timestamp_after_process"] = ts_after

            logging.debug(f"Processing took {(ts_after-ts_before)/10**9}")

            assert len(detections) == len(images)

            for idx in range(len(detections)):
                self.publish_results(detections[idx], metadata[idx])

    def publish_results(self, results, metadata):
        """
        Publishes the results to the robot

        Args:
            results (_type_): The results of the detection.
            metadata (_type_): NetApp-specific metadata related to processed image.
        """
        detections = list()
        if results is not None:
            for bbox, score, cls_id, cls_name in results:
                det = dict()
                det["bbox"] = [float(i) for i in bbox]
                det["score"] = float(score)
                det["class"] = int(cls_id)
                det["class_name"] = str(cls_name)

                detections.append(det)

            send_timestamp = time.time_ns()

            # TODO:check timestamp exists
            r = {
                "timestamp": metadata["timestamp"],
                "recv_timestamp": metadata["recv_timestamp"],
                "timestamp_before_process": metadata["timestamp_before_process"],
                "timestamp_after_process": metadata["timestamp_after_process"],
                "send_timestamp": send_timestamp,
                "detections": detections,
            }

            # TODO figure out if emit is blocking or not (if so, it should be called from a separate thread)
            self.sio.emit(
                "message", r, namespace="/results", to=metadata["websocket_id"]
            )
