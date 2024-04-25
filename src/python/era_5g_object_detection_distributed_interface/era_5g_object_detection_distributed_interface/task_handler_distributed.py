
from queue import Full, Queue

from era_5g_object_detection_distributed_interface.worker_interface import detector_task


class TaskHandlerCelery():
    """
    Task handler which stores the retrieved images to the message broker queue.
    """

    def __init__(self, jobs_info_queue: Queue, **kw):
        """
        Constructor

        Args:
            jobs_info_queue (Queue): Additional queue for passing information about all to-be-processed jobs
                to the results reader.
        """

        super().__init__(**kw)
        self._jobs_info_queue = jobs_info_queue

    def store_image(self, metadata, image):
        """
        Method which creates a remote processing task using Celery.

        Args:
            metadata (Dict): Arbitrary dictionary with metadata related to the image.
                The format is 5G-ERA Network Application specific.
            image (_type_): The image to be processed.
        """

        job_data = (metadata, image)

        # Create asynchronous Celery task
        job = detector_task.delay(job_data)

        try:
            # Store the job information (pass it to the results reader)
            self._jobs_info_queue.put(job, block=False)
        except Full:
            job.revoke()
            return

