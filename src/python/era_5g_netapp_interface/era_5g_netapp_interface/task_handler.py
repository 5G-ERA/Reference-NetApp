from threading import Thread
from abc import abstractmethod, ABCMeta


class TaskHanlderInitializationFailed(Exception):
    pass

class TaskHandler(Thread):
    """
    Abstract class. Thread-based task handler which takes care
    of recieving data from the NetApp client and passing them 
    to the NetApp worker.

    """
    __metaclass__ = ABCMeta

    def __init__(self, logger, sid):
        super().__init__()
        self.sid = sid
        self.websocket_id = None
        self.logger = logger

    def start(self, daemon):
        self.logger.debug("Starting %s thread", self.name)
        t = Thread(target=self._run, args=())
        t.daemon = daemon
        t.start()

    def stop(self):
        self.logger.debug("Stopping %s thread", self.name)
        self.stopped = True

    @abstractmethod
    def _run(self):
        """
        This method is run once the thread is starded and could be used
        for periodical retrival of images.
        """
        pass

    @abstractmethod
    def store_image(self, metadata: dict, image):
        """
        This method is intended to pass the image to the worker (using internal queues,
        message broker or anything else).

        Args:
            metadata (dict): Arbitrary dictionary with metadata related to the image.
                The format is NetApp-specific.
            image (_type_): The image to be processed.
        """
        pass


