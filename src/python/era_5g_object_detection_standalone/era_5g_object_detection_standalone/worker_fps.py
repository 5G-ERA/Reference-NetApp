from queue import Queue

from era_5g_object_detection_common.fps_test_detector import FpsTestDetector
from era_5g_object_detection_standalone.worker import Worker


class FpsDetectorWorker(Worker, FpsTestDetector):
    """
    Worker object for the debug detector which returns the framerate of received stream.
    """

    def __init__(self, image_queue: Queue, sio, **kw):
        """
        Constructor

        Args:
            image_queue (Queue): Queue with all to-be-processed images.
            app (_type_): A flask app for results publishing.
        """

        super().__init__(image_queue=image_queue, sio=sio, **kw)
