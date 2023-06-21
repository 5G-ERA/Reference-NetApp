from queue import Queue

from era_5g_object_detection_common.face_detector import FaceDetector
from era_5g_object_detection_standalone.worker import Worker


class FaceDetectorWorker(Worker, FaceDetector):
    """
    Worker object for the basic face detector based on OpenCV Haar-cascade Detection. 
    """

    def __init__(self, image_queue: Queue, sio, **kw):
        """
        Constructor

        Args:
            image_queue (Queue): Queue with all to-be-processed images.
            app (_type_): A flask app for results publishing.
        """

        super().__init__(image_queue=image_queue, sio=sio, **kw)
