from queue import Full, Queue

from era_5g_netapp_interface.task_handler import TaskHandler


class TaskHandlerInternalQ(TaskHandler):
    """
    Task handler which takes care of passing the data to the python 
    internal queue for future processing. It could either be inherited
    to implement the _run method and read the data from any source or used
    directly and call the store_image method externally.
    """

    def __init__(self, sid: str, image_queue: Queue, **kw):
        """
        Constructor

        Args:
            sid (str): The session id obtained from NetApp client. It is used to 
                match the results with the data sender.
            image_queue (Queue): The queue where the image and metadata should 
                be passed to.
        """

        super().__init__(sid=sid, **kw)
        self._q = image_queue
        self.index = 0

    def store_image(self, metadata: dict, image):
        """
        Method which will store the image to the queue for processing.

        Args:
            metadata (dict): Arbitrary dictionary with metadata related to the image.
                The format is NetApp-specific.
            image (_type_): The image to be processed.
        """

        try:
            self._q.put((metadata, image), block=False)
        except Full:
            pass
            # TODO: raise an exception

    def run(self):
        pass
