from abc import abstractmethod, ABC
from threading import Thread, Event
from typing import List, Tuple
import numpy as np

from era_5g_interface.interface_helpers import LatencyMeasurements

BasicDetectorResultType = List[Tuple[List[float], float, int, str]]


class ImageDetectorInitializationFailed(Exception):
    pass


class ImageDetector(Thread, ABC):
    """
    The base class for NetApps based on image processing. Provides abstract
    methods for processing images and publishing results. It is based on Threads.
    """

    def __init__(self, **kw):
        super().__init__(**kw)
        self.stop_event = Event()
        self.time = None
        self.fps = 0.0
        self.latency_measurements = LatencyMeasurements()

    def stop(self):
        self.stop_event.set()

    @abstractmethod
    def run(self):
        """
        This method is run once the thread is started and could be used e.g.
        for periodical retrieval of images.
        """

        pass

    @abstractmethod
    def process_image(self, frame: np.array):
        """
        This method is responsible for processing of passed image.

        Args:
            frame (np.array): Image to be processed

        Raises:
            NotImplemented
        """

        pass

    @abstractmethod
    def process_images(self, frames: List[np.array]):
        """
        This method is responsible for processing of passed image.

        Args:
            frames (List[np.array]): Images to be processed

        Raises:
            NotImplemented
        """

        pass

    @abstractmethod
    def publish_results(self, **kw):
        """
        This method is responsible for returning results back to the robot. 

        Raises:
            NotImplemented
        """

        pass
