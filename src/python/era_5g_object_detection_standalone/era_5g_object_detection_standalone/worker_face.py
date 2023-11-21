from typing import Callable, Dict
from queue import Queue

from era_5g_object_detection_common.face_detector import FaceDetector
from era_5g_object_detection_standalone.worker import Worker


class FaceDetectorWorker(Worker, FaceDetector):
    """Worker object for the basic face detector based on OpenCV Haar-cascade Detection."""

    def __init__(self, image_queue: Queue, send_function: Callable[[Dict], None], **kw):
        """Constructor.

        Args:
            image_queue (Queue): Queue with all to-be-processed images.
            send_function (Callable[[Dict], None]): Callback used to send results.
        """

        super().__init__(image_queue=image_queue, send_function=send_function, **kw)
