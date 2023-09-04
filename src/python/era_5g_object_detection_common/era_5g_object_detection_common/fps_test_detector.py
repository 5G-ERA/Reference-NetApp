import time
from abc import ABC

import numpy as np
from typing import List

from era_5g_object_detection_common.image_detector import ImageDetector, BasicDetectorResultType


class FpsTestDetector(ImageDetector, ABC):
    """
    Debug detector which returns the framerate of received stream.
    """

    def __init__(self, **kw):
        """
        Constructor

        Args:
            name (str): The name of the thread.
        """

        super().__init__(**kw)
        print("fps_test_detector")
        self.start_time = time.time()
        self.frames = 0

    def process_image(self, frame: np.array) -> BasicDetectorResultType:
        """
        Counts the number of frames per seconds and returns the value
        each second.

        Args:
            frame (np.array): The received image

        Returns:
            list(tuple(bbox[], fps_value, class_id, class_name)): Number of received
                frames per second (once per second, empty list otherwise)
        """

        # Detect the faces
        self.frames += 1
        if time.time() - self.start_time > 1:
            fps = self.frames / (time.time() - self.start_time)
            print(f"FPS: {fps}")
            self.frames = 0
            self.start_time = time.time()
            return [([0, 0, 0, 0], fps, 0, "fps")]
        
    def process_images(self, frames: List[np.array]) -> List[BasicDetectorResultType]:
        """
        Wrapper around process_image to simulate batched processing.

        Args:
            frames (list(np.array)): Batch with frames

        Returns:
            list(list(tuple(bbox[], fps_value, class_id, class_name))):
                List with values that would be returned by individual calls to
                process_image.
        """
        
        return [self.process_image(frame) for frame in frames]
