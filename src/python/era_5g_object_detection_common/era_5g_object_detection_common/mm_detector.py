import os
from abc import ABC
from typing import Dict, List, Any

import numpy as np
from mmdet.apis import init_detector, inference_detector

from era_5g_object_detection_common.image_detector import (
    ImageDetector,
    ImageDetectorInitializationFailed,
    BasicDetectorResultType,
)
from era_5g_object_detection_common.mmdet_utils import MODEL_VARIANTS, convert_mmdet_result


class MMDetector(ImageDetector, ABC):
    """Universal detector based on MMDET package."""

    def __init__(self, **kw):
        """Constructor.

        Raises:
            ImageDetectorInitializationFailed: Raised when initialization failed, e.g. when required parameter is not
                set.
        """

        super().__init__(**kw)
        # Path to the folder where the mmdet module is installed.
        self.path_to_mmdet = os.getenv("NETAPP_MMDET_PATH", None)
        # The selected model variant, possible values are listed in era_5g_object_detection_common.mmdet_utils.
        self.model_variant = os.getenv("NETAPP_MODEL_VARIANT", None)
        # The device where the inference takes a place ('cpu', 'cuda', 'cuda:0', etc.).
        self.torch_device = os.getenv("NETAPP_TORCH_DEVICE", "cpu")
        if not self.path_to_mmdet:
            raise ImageDetectorInitializationFailed(
                f"Failed to load mmdet module, env variable NETAPP_MMDET_PATH not set"
            )
        if not os.path.exists(self.path_to_mmdet):
            raise ImageDetectorInitializationFailed(
                f"Failed to load mmdet module, path {self.path_to_mmdet} does not exist"
            )
        elif not self.model_variant:
            raise ImageDetectorInitializationFailed(f"Failed to load model, env variable NETAPP_MODEL_VARIANT not set")
        config_file = os.path.join(self.path_to_mmdet, MODEL_VARIANTS[self.model_variant]["config_file"])
        checkpoint_file = os.path.join(self.path_to_mmdet, MODEL_VARIANTS[self.model_variant]["checkpoint_file"])
        self.model = init_detector(config_file, checkpoint_file, device=self.torch_device)

    def process_image(self, frame: np.array) -> BasicDetectorResultType:
        """Detects the objects of selected classes the incoming frame and returns all detections.

        Args:
            frame (np.array): The passed image.

        Returns:
            BasicDetectorResultType: The list of detected objects, with bounding box (x1, y1, x2, y2,
            top-left bottom-right corners), score (0..1), class_id and class_name.
        """
        if frame is not None:
            # Gets results from detector.
            result = inference_detector(self.model, frame)
            # Convert results to the standard format and return it to the robot.
            return convert_mmdet_result(
                result, merged_data=True, with_mask=MODEL_VARIANTS[self.model_variant]["with_masks"]
            )
        else:
            # TODO: raise an exception
            return []

    def process_images(self, frames: List[np.array]) -> List[BasicDetectorResultType]:
        """
        Detects the objects of selected classes the incoming frame and returns all detections.

        Args:
            frames (List[np.array]): The passed images.

        Returns:
            List[BasicDetectorResultType]: The list of detected objects, with bounding box (x1, y1, x2, y2,
            top-left bottom-right corners), score (0..1), class_id and class_name.
        """
        if frames is not None:
            # Gets results from detector.
            result = inference_detector(self.model, frames)
            # Convert results to the standard format and return it to the robot.
            converted = []
            for r in result:
                converted.append(
                    convert_mmdet_result(
                        r, merged_data=True, with_mask=MODEL_VARIANTS[self.model_variant]["with_masks"]
                    )
                )

            return converted
        else:
            # TODO: raise an exception
            return []

    def prepare_detections_for_publishing(self, results: BasicDetectorResultType) -> Dict[str, Any]:
        """Convert results to a representation that is ready for publishing."""

        detections = list()
        for result in results:
            det = dict()
            # Process the results based on currently used model.
            if MODEL_VARIANTS[self.model_variant]["with_masks"]:
                bbox, score, cls_id, cls_name, mask = result
                det["mask"] = mask
            else:
                bbox, score, cls_id, cls_name = result
            det["bbox"] = [float(i) for i in bbox]
            det["score"] = float(score)
            det["class"] = int(cls_id)
            det["class_name"] = str(cls_name)

            detections.append(det)
        return detections
