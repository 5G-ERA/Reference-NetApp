from abc import abstractmethod, ABC
from threading import Thread, Event


class TaskHandlerInitializationFailed(Exception):
    pass


class TaskHandler(Thread, ABC):
    """
    Abstract class. Thread-based task handler which takes care
    of receiving data from the NetApp client and passing them
    to the NetApp worker.
    """

    def __init__(self, sid: str, **kw):
        super().__init__(**kw)
        self.stop_event = Event()
        self.sid = sid
        self.websocket_id = None

    def stop(self):
        self.stop_event.set()

    @abstractmethod
    def run(self):
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
