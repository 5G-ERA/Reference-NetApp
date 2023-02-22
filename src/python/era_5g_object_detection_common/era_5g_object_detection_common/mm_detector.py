import json
import os
from abc import ABC
from queue import Empty, Queue
import cv2
import flask_socketio
import numpy as np

from era_5g_object_detection_common.image_detector import ImageDetector, ImageDetectorInitializationFailed
from era_5g_object_detection_common.mmdet_utils import MODEL_VARIANTS, convert_mmdet_result
from mmdet.apis import init_detector, inference_detector


class MMDetector(ImageDetector, ABC):
    """
    Universal detector based on MMDET package.

    """

    def __init__(self, **kw):
        """
        Constructor

        Args:
            name (str): The name of the thread.

        Raises:
            ImageDetectorInitializationFailed: Raised when initialization failed, e.g. when 
                required parameter is not set.
        """

        super().__init__(**kw)
        # path to the folder where the mmdet module is installed
        self.path_to_mmdet = os.getenv("NETAPP_MMDET_PATH", None)
        # the selected model variant, possible values are listed in era_5g_object_detection_common.mmdet_utils
        self.model_variant = os.getenv("NETAPP_MODEL_VARIANT", None)
        # the device where the inference takes a place ('cpu', 'cuda', 'cuda:0', etc.)
        self.torch_device = os.getenv("NETAPP_TORCH_DEVICE", 'cpu')
        if not self.path_to_mmdet:
            raise ImageDetectorInitializationFailed(
                f"Failed to load mmdet module, env variable NETAPP_MMDET_PATH not set")
        if not os.path.exists(self.path_to_mmdet):
            raise ImageDetectorInitializationFailed(
                f"Failed to load mmdet module, path {self.path_to_mmdet} does not exist")
        elif not self.model_variant:
            raise ImageDetectorInitializationFailed(f"Failed to load model, env variable NETAPP_MODEL_VARIANT not set")
        config_file = os.path.join(self.path_to_mmdet, MODEL_VARIANTS[self.model_variant]['config_file'])
        checkpoint_file = os.path.join(self.path_to_mmdet, MODEL_VARIANTS[self.model_variant]['checkpoint_file'])
        self.model = init_detector(config_file, checkpoint_file, device=self.torch_device)

    def process_image(self, frame):
        """
        Detects the objects of selected classes the incoming frame and returns all detections.

        Args:
            frame (_type_): The passed image

        Returns:
            list(tuple(bbox[], score, class_id, class_name)): The list of detected objects,
            with bounding box (x1, y1, x2, y2, top-left bottom-right corners), score (0..1), 
            class_id and class_name.
        """

        if frame is not None:
            # gets results from detector
            result = inference_detector(self.model, frame)
            # convert results to the standard format and return it to the robot
            return convert_mmdet_result(result, merged_data=True,
                                        with_mask=MODEL_VARIANTS[self.model_variant]['with_masks'])
        else:
            # TODO: raise an exception
            return []
