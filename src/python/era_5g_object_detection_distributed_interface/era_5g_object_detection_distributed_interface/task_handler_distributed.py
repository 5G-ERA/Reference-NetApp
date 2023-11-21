from threading import Thread
from queue import Full, Queue
import cv2
from era_5g_interface.task_handler import TaskHandler
from era_5g_object_detection_distributed_interface.worker_interface import detector_task
from abc import ABC


class TaskHandlerDistributed(TaskHandler, ABC):
    """
    Task handler which stores the retrieved images to the RabbitMQ broker.
    """

    def __init__(self, sid: str, jobs_info_queue: Queue, **kw):
        """
        Constructor

        Args:
            sid (str): The session id obtained from 5G-ERA Network Application client. It is used to
                match the results with the data sender.
            port (int): The port where the Gstreamer pipeline should listen to.
            jobs_info_queue (Queue): Queue with all to-be-processed jobs.
        """

        super().__init__(sid, **kw)
        self._jobs_info_queue = jobs_info_queue

    def store_image(self, metadata, image):
        """
        Method which will call the remote processing using Celery and RabbitMQ.

        Args:
            metadata (Dict): Arbitrary dictionary with metadata related to the image.
                The format is 5G-ERA Network Application specific.
            image (_type_): The image to be processed.
        """

        if metadata.get("decoded", True):
            job_data = (metadata, image)
        else: 
            # decode image
            img = cv2.imdecode(image, cv2.IMREAD_COLOR)            
            job_data = (metadata, img)
        # Start asynchronous Celery task
        job = detector_task.delay(job_data)
        try:
            # store the job information
            self._jobs_info_queue.put(job, block=False)
        except Full:
            job.revoke()
            return

    def run(self) -> None:
        pass
