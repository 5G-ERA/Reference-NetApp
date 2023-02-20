from queue import Queue
from logging import Logger

from era_5g_netapp_interface.task_handler_gstreamer import TaskHandlerGstreamer
from era_5g_netapp_interface.task_handler_internal_q import TaskHandlerInternalQ


class TaskHandlerGstreamerInternalQ(TaskHandlerGstreamer, TaskHandlerInternalQ):
    """
    Task handler which combines the Gstreamer functionality of data retrieval
    with usage of python internal queues for passing the data to the worker object.
    """

    def __init__(self, logger: Logger, sid: str, port: str, image_queue: Queue):
        """
        Constructor

        Args:
            logger (Logger): A thread-safe logger. Could be obtained using the
                era_5g_netapp_interface.common.get_logger() function.
            sid (str): The session id obtained from NetApp client. It is used to 
                match the results with the data sender.
            port (str): The port where the Gstreamer pipeline should listen to.
            image_queue (Queue): The queue where the image and metadata should 
                be passed to.
        """

        super().__init__(logger=logger, sid=sid, port=port, image_queue=image_queue)
      