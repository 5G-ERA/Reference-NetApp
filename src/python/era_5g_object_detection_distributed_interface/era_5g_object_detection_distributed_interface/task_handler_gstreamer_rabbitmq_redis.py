from threading import Thread
from queue import Full, Queue
from era_5g_netapp_interface.task_handler_gstreamer import TaskHandlerGstreamer
from era_5g_object_detection_distributed_worker import detector_task


class TaskHandlerGstreamerRabbitmqRedis(TaskHandlerGstreamer):
    """
    Task handler which stores the retrieved images to the RabbitMQ broker.

    """

    def __init__(self, logger, sid, port: str, jobs_info_queue: Queue):
        """
        Constructor

        Args:
            logger (_type_): A thread-safe logger. Could be obtained using the 
                era_5g_netapp_interface.common.get_logger() function.
            sid (str): The session id obtained from NetApp client. It is used to 
                match the results with the data sender.
            port (str): The port where the Gstreamer pipeline should listen to.
            jobs_info_queue (Queue): Queue with all to-be-processed jobs.
        """
        super().__init__(logger, sid, port)
        self._jobs_info_queue = jobs_info_queue

    def store_image(self, metadata, image):
        """
        Method which will call the remote processing using Celery and RabbitMQ.

        Args:
            metadata (dict): Arbitrary dictionary with metadata related to the image.
                The format is NetApp-specific.
            image (_type_): The image to be processed.
        """
        job_data = (metadata, image)
        # Start asynchronous Celery task
        job = detector_task.delay(job_data)
        try:
            # store the job information
            self._jobs_info_queue.put(job, block=False)
        except Full:
            job.revoke()
            return

    

