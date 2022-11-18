from queue import Queue

from era_5g_object_detection_common.fps_test_detector import FpsTestDetector
from era_5g_object_detection_standalone.worker import Worker


class FpsDetectorWorker(Worker, FpsTestDetector):
    """
    Worker object for the debug detector which returns the framerate of recieved stream.

    
    """
    def __init__(self, logger, name:str, image_queue: Queue, app):
        """
        Constructor

        Args:
            logger (_type_): A thread-safe logger. Could be obtained using the 
                era_5g_netapp_interface.common.get_logger() function.
            name (str): The name of the thread.
            image_queue (Queue): Queue with all to-be-processed images.
            app (_type_): A flask app for results publishing.
        """
        super().__init__(logger=logger, name=name, image_queue=image_queue, app=app)

    

    
        
