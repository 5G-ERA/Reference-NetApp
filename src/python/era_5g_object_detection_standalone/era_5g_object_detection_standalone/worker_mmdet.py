from queue import Queue
import flask_socketio

from era_5g_object_detection_common.mm_detector import MMDetector
from era_5g_object_detection_standalone.worker import Worker
from era_5g_object_detection_common.mmdet_utils import MODEL_VARIANTS


class MMDetectorWorker(Worker, MMDetector):
    """
    Worker object for the universal detector based on MMDET package.

    
    """
    def __init__(self, logger, name, image_queue: Queue, app):
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

    def publish_results(self, results, metadata):
        """
        Publishes the results to the robot

        Args:
            metadata (_type_): NetApp-specific metadata related to processed image. TODO: describe the metadata
            results (_type_): The results of the detection. TODO: describe the results format
        """
        detections = list()

        det = dict()
        for result in results:
            # process the results based on currently used model
            if MODEL_VARIANTS[self.model_variant]['with_masks']:
                bbox, score, cls_id, cls_name, mask = result
                det["mask"] = mask
            else:
                bbox, score, cls_id, cls_name = result
            det["bbox"] = [float(i) for i in bbox]
            det["score"] = float(score)
            det["class"] = int(cls_id)
            det["class_name"] = str(cls_name)
            
            detections.append(det)
        # add timestamp to the results
        r = {"timestamp": metadata["timestamp"],
                "detections": detections}
            
        # use the flask app to return the results
        with self.app.app_context():              
                flask_socketio.send(r, namespace='/results', to=metadata["websocket_id"])