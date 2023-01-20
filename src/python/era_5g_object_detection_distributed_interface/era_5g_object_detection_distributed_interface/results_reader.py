import json
from queue import Empty, Queue
import string
from time import sleep
from era_5g_netapp_interface.common import ThreadBase
import flask_socketio

class ResultsReader(ThreadBase):
    """
    Thread-based class which search the jobs queue for finished tasks and
    publish results to the robot 

    
    """
    def __init__(self, logger, name: str, jobs_info_queue: Queue, app):
        """
        Constructor

        Args:
            logger (_type_): A thread-safe logger. Could be obtained using the 
                era_5g_netapp_interface.common.get_logger() function.
            name (str): Name of the thread
            jobs_info_queue (Queue): Queue with all to-be-processed jobs.
            app (_type_): A flask app for results publishing.
        """
        super().__init__(logger, name)

        self.stopped = False
        self.time = None
        self.fps = 0.0
        self.logger = logger
        self._jobs_info_queue = jobs_info_queue
        self.app = app
        
        self.jobs_in_process = []
        

    def _run(self):
        """
        Periodically reads the queue and search for finished jobs.
        """
        self.logger.info(f"Results reader thread for {self.name} is running.")

        while not self.stopped:
            # Check for newly created jobs
            try:
                job = self._jobs_info_queue.get_nowait()
                self.jobs_in_process.append(job)
            except Empty:
                pass
            
            # Check for completed jobs
            jobs_to_remove = set()
            for job in self.jobs_in_process:
                
                if job.state == "SUCCESS":
                    jobs_to_remove.add(job)
                    (metadata, results) = job.get()
                    self.publish_results(metadata, results)
                    
                elif job.state == "REVOKED":
                    jobs_to_remove.add(job)

                elif job.state == "FAILURE":
                    jobs_to_remove.add(job)
                    self.logger.info(f"Task {job.task_id} failed.")
                    # TODO: optional error handling
                        
            # Remove completed jobs
            if len(jobs_to_remove):
                self.jobs_in_process = [job for job in self.jobs_in_process if job not in jobs_to_remove]
            else: 
                sleep(0.02)
        
        print(f"job called: {job}")
    def publish_results(self, metadata, results):
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
        # adds timestamp to the results
        r = {"timestamp": metadata["timestamp"],
                "detections": detections}
            
        # use the flask app to return the results
        with self.app.app_context():              
                flask_socketio.send(r, namespace='/results', to=metadata["websocket_id"])
        
       