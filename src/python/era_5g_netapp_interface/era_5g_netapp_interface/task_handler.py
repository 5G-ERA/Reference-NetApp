from abc import abstractmethod, ABC
from logging import Logger

from era_5g_netapp_interface.common import ThreadBase


class TaskHandlerInitializationFailed(Exception):
    pass


class TaskHandler(ThreadBase, ABC):
    """
    Abstract class. Thread-based task handler which takes care
    of receiving data from the NetApp client and passing them
    to the NetApp worker.
    """

    def __init__(self, logger: Logger, sid: str):
        super().__init__(logger=logger, name=sid)
        self.sid = sid
        self.websocket_id = None
        self.logger = logger

    @abstractmethod
    def _run(self):
        """
        This method is run once the thread is started and could be used
        for periodical retrieval of images.
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
