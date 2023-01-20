from asyncio import Queue

from era_5g_netapp_interface.common import ThreadBase


class ImageDetectorInitializationFailed(Exception):
    pass


class ImageDetector(ThreadBase):
    """
    The base class for NetApps based on image processing. Provides abstract
    methods for processing images and publishing results. It is based on Threads.

    """
    def __init__(self, logger, name: str):
        """
        Constructor

        Args:
            logger (_type_): A thread-safe logger. Could be obtained using the 
                era_5g_netapp_interface.common.get_logger() function.
            name (str): The name of the thread.
        """
        super().__init__(logger, name)
        self.stopped = False
        self.time = None
        self.fps = 0.0

    def run(self):
        """
        This method is run once the thread is starded and could be used e.g.
        for periodical retrival of images.
        """
        pass

    def process_image(self, frame):
        """
        This method is responsible for processing of passed image.

        Args:
            frame (_type_): Image to be processed

        Raises:
            NotImplemented
        """
        raise NotImplemented()

    def publish_results(self, data: any):
        """
        This method is responsible for returning results back to the robot. 

        Args:
            data (_type_): The results to be returned to the robot. The format is
                NetApp-specific.

        Raises:
            NotImplemented
        """
        raise NotImplemented()
        