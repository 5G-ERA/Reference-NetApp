import os
import numpy as np

from mmdet.core import get_classes

DEBUG=True

# mmDetection model config and checkpoint files (=allowed values of NETAPP_MODEL_VARIANT env variable)
MODEL_VARIANTS = {
    'yolov3_mobilenet': {
        'config_file': 'configs/yolo/yolov3_mobilenetv2_320_300e_coco.py',
        'checkpoint_file': 'configs/yolo/yolov3_mobilenetv2_320_300e_coco_20210719_215349-d18dff72.pth',
        'with_masks': False
    },
    'cv2_faces': None,  # Only added so that a tag name exists for the old opencv face detector
    
    # blank example:
    #'model_variant_name': {
    #    'config_file': '',
    #    'checkpoint_file': '',
    #    'with_masks': True
    #},
}

def convert_mmdet_result(result, dataset='coco', score_thr=0.5, with_mask=False, merged_data=True):
    # Convert raw results from mmDet to a desired format.
    # inspired by:
    # https://github.com/open-mmlab/mmdetection/issues/248#issuecomment-454276078
    # and
    # https://vinleonardo.com/detecting-objects-in-pictures-and-extracting-their-data-using-mmdetection/

    segm_result = None
    if with_mask:
        bbox_result, segm_result = result
    else:
        bbox_result = result

    class_ids_raw = [
        np.full(bbox.shape[0], i, dtype=np.int32) \
        for i, bbox in enumerate(bbox_result)
    ]
    class_ids_raw = np.concatenate(class_ids_raw)
    bboxes_with_scores = np.vstack(bbox_result)
    scores_raw = bboxes_with_scores[:, -1]
    bboxes_raw = bboxes_with_scores[:, :-1]
    filtered_inds = np.where(scores_raw > score_thr)[0]
    bboxes = bboxes_raw[filtered_inds]
    scores = scores_raw[filtered_inds]

    all_class_names = get_classes(dataset)
    class_ids = [class_ids_raw[i] for i in filtered_inds]
    class_names = [all_class_names[i] for i in class_ids]

    if with_mask:
        pass  # TODO

    if merged_data:
        converted_result = list(zip(bboxes, scores, class_ids, class_names))
    else:
        converted_result = {"bboxes": bboxes, "scores": scores, "class_ids": class_ids, "class_names": class_names}
    return converted_result
