from abc import ABC
from multiprocessing import Queue
from queue import Empty
import logging
import cv2
import time

from era_5g_object_detection_common.image_detector import ImageDetector


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

    def run(self):
        """
        Periodically reads images from python internal queue process them.
        """

        logging.info(f"{self.name} thread is running.")
    
        while not self.stop_event.is_set():
            # Get image and metadata from input queue
            try:
                metadata1, image1 = self.image_queue.get(block=True, timeout=1)
                metadata2, image2 = self.image_queue.get(block=True, timeout=1)
                metadata3, image3 = self.image_queue.get(block=True, timeout=1)
                metadata3, image4 = self.image_queue.get(block=True, timeout=1)
                metadata3, image5 = self.image_queue.get(block=True, timeout=1)
            except Empty:
                continue
            metadata1["timestamp_before_process"] = time.time_ns()
            if metadata1.get("decoded", True):
                detections = self.process_images((image1, image2, image3, image4, image5))
                #print(detections)
                
            else: 
                # decode image
                img = cv2.imdecode(image, cv2.IMREAD_COLOR)
                detections = self.process_image(img)
            #metadata["timestamp_after_process"] = time.time_ns()
            #print(detections)
            for d in detections:
                #print("before p")
                self.publish_results(d, metadata1)
        
            

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

            send_timestamp = time.time_ns()

            # TODO:check timestamp exists
            r = {"timestamp": metadata["timestamp"],
                 "recv_timestamp": metadata["recv_timestamp"],
                 "timestamp_before_process": metadata["timestamp_before_process"],
                 "timestamp_after_process": metadata["timestamp_after_process"],
                 "send_timestamp": send_timestamp,
                 "detections": detections}
            
            self.sio.emit('message', r, namespace="/results", to=metadata["websocket_id"])