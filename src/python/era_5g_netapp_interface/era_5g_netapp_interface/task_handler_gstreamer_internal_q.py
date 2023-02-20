from queue import Queue

from era_5g_netapp_interface.task_handler_gstreamer import TaskHandlerGstreamer
from era_5g_netapp_interface.task_handler_internal_q import TaskHandlerInternalQ


class TaskHandlerGstreamerInternalQ(TaskHandlerGstreamer, TaskHandlerInternalQ):
    """
    Task handler which combines the Gstreamer functionality of data retrieval
    with usage of python internal queues for passing the data to the worker object.
    """

    def __init__(self, sid: str, port: int, image_queue: Queue, **kw):
        """
        Constructor

        Args:
            sid (str): The session id obtained from NetApp client. It is used to 
                match the results with the data sender.
            port (int): The port where the Gstreamer pipeline should listen to.
            image_queue (Queue): The queue where the image and metadata should 
                be passed to.
        """

        super().__init__(sid=sid, port=port, image_queue=image_queue, **kw)
      