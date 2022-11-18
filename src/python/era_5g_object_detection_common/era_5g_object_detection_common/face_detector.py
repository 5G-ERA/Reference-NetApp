import os

import cv2
import numpy as np
from era_5g_object_detection_common.image_detector import ImageDetector, ImageDetectorInitializationFailed

# path to the face detector model file (the haarcascade xml)
MODEL_FILE = os.getenv("NETAPP_FACE_DETECTOR_MODEL_FILE", None)

class FaceDetector(ImageDetector):
    """
    Basic face detector based on OpenCV Haar-cascade Detection. 

    """
    def __init__(self, logger, name, **kw):
        """
        Constructor

        Args:
            logger (_type_): A thread-safe logger. Could be obtained using the 
                era_5g_netapp_interface.common.get_logger() function.
            name (str): The name of the thread.

        Raises:
            ImageDetectorInitializationFailed: Raised when initialization failed, e.g. when 
                required parameter is not set.
        """
        super().__init__(logger, name)
        print("face_detector")
        if MODEL_FILE is None:
            raise ImageDetectorInitializationFailed("Failed to initialize detector, env variable NETAPP_FACE_DETECTOR_MODEL_FILE not set")
        elif not os.path.exists(MODEL_FILE):
            raise ImageDetectorInitializationFailed(f"Failed to initialize detector, {MODEL_FILE} does not exist!")
        self.detection_cascade = cv2.CascadeClassifier(MODEL_FILE)

   
    def process_image(self, frame):
        """
        Detects the faces in the passed frame and returns all detected faces.

        Args:
            frame (_type_): The passed image

        Returns:
            list(tuple(bbox[], score, class_id, class_name)): The list of detected faces,
            with bounding box (x1, y1, x2, y2, top-left bottom-right corners), score (0..1), 
            class_id (equals to 1 for faces) and class_name ("face").
        """
        # Detect the faces
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = self.detection_cascade.detectMultiScale(gray, 1.35, 4)

        detections_raw = []
        
        for bbox in faces:
            
            # Transform from x,y,w,h to x1,y1,x2,y2 (bbox top-left bottom-right corners)
            bbox[2] += bbox[0] 
            bbox[3] += bbox[1]

            # Assigne appropriete class id and name
            cls = 1  
            cls_name = "face"

            score = 0 # opencv detector does not provide score
            det = bbox, score, cls, cls_name

            # Add to other detections for processed frame
            detections_raw.append(det)        

        return detections_raw

    
        
