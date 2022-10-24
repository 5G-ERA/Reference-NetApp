import os
import numpy as np
import base64

import mmcv
import pycocotools.mask as masks_util
from mmdet.core import get_classes

DEBUG=True

# mmDetection model config and checkpoint files (=allowed values of NETAPP_MODEL_VARIANT env variable)
MODEL_VARIANTS = {
    'yolov3_mobilenet': {
        'config_file': 'configs/yolo/yolov3_mobilenetv2_320_300e_coco.py',
        'checkpoint_file': 'configs/yolo/yolov3_mobilenetv2_320_300e_coco_20210719_215349-d18dff72.pth',
        'with_masks': False
    },

    'cv2_faces': { # Only added so that a tag name exists for the old opencv face detector
        'with_masks': False,  
    },

    'mask_rcnn_r50': {
        'config_file': 'configs/mask_rcnn/mask_rcnn_r50_caffe_fpn_mstrain-poly_3x_coco.py',
        'checkpoint_file': 'configs/mask_rcnn/mask_rcnn_r50_caffe_fpn_mstrain-poly_3x_coco_bbox_mAP-0.408__segm_mAP-0.37_20200504_163245-42aa3d00.pth',
        'with_masks': True
    },
    
    # Example:
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
        # original results have length of num_classes, and then for each class there are individual detections
        all_masks = np.array([item for sublist in segm_result for item in sublist])  # flatten the structure
        filtered_masks = all_masks[filtered_inds]  # filter by given confidence threshold

        # Encode (compress) masks using RLE
        # inspired by encode_mask_results function:
        # https://mmdetection.readthedocs.io/en/latest/_modules/mmdet/core/mask/utils.html
        # -> can be used on the original data structure: encoded_masks = encode_mask_results(segm_result)
        # internally uses pycocotools: https://github.com/cocodataset/cocoapi/blob/master/PythonAPI/pycocotools/mask.py
        encoded_masks = []
        for cls_segm in filtered_masks:
            # each mask is transformed into a dict with format: {'size': [h, w], 'counts': b'encoded_mask_binary_data'}
            mask = masks_util.encode(np.array(cls_segm[:, :, np.newaxis], order='F', dtype='uint8'))[0]  # RLE
            encoded_masks.append(mask)

        # Base64 encoding
        for mask in encoded_masks:
            mask_data = mask["counts"]
            b64enc_data_bin = base64.b64encode(mask_data)  # returns base64 encoded data in binary format
            b64end_data_str = b64enc_data_bin.decode('ascii')  # converts base64 encoded binary data to string
            mask["counts"] = b64end_data_str

    if merged_data:
        if with_mask:
            converted_result = list(zip(bboxes, scores, class_ids, class_names, encoded_masks))
        else:
            converted_result = list(zip(bboxes, scores, class_ids, class_names))
    else:
        converted_result = {"bboxes": bboxes, "scores": scores, "class_ids": class_ids, "class_names": class_names}
        if with_mask:
            converted_result["masks"] = encoded_masks

    return converted_result
