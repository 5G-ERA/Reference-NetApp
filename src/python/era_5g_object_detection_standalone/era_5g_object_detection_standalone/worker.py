from queue import Empty, Queue
import flask_socketio
from era_5g_object_detection_common.image_detector import ImageDetector
import cv2

class Worker(ImageDetector):
    """
    Worker object for image processing in standalone variant. Reads 
    data from passed queue, performs detection and returns results using
    the flask app. Needs to be inherited to implement the process_image
    method.

    """

    def __init__(self, image_queue: Queue, app, **kw):
        """
        Constructor

        Args:
            image_queue (Queue): The queue with all to-be-processed images
            app (_type_): The flask app for results publishing
        """
        super().__init__(**kw)
        self.image_queue = image_queue
        self.app = app

    def run(self):
        """
        Periodically reads images from python internal queue process them.
        """
        self.logger.debug(f"{self.name} thread is running.")
        
        while not self.stopped:
            # Get image and metadata from input queue
            try:
                metadata, image = self.image_queue.get(block=True)
            except Empty:
                continue
            
            detections = self.process_image(image)
            
            self.publish_results(detections, metadata)
            


    def publish_results(self, results, metadata):
        """
        Publishes the results to the robot

        Args:
            metadata (_type_): NetApp-specific metadata related to processed image.
            results (_type_): The results of the detection.
        """
        detections = list()
        for (bbox, score, cls_id, cls_name) in results:
            
            det = dict()
            det["bbox"] = [float(i) for i in bbox]
            det["score"] = float(score)
            det["class"] = int(cls_id)
            det["class_name"] = str(cls_name)
            
            detections.append(det)
        
        r = {"timestamp": metadata["timestamp"],
                "detections": detections}
            
        # use the flask app to return the results
        with self.app.app_context():              
                flask_socketio.send(r, namespace='/results', to=metadata["websocket_id"])