#!/bin/bash
#ROOT_PATH=$(dirname "$BASH_SOURCE")
ROOT_PATH=$(dirname $(realpath "$BASH_SOURCE"))
ASSETS_PATH="$ROOT_PATH/assets/"
export ROS2_5G_ERA_ASSETS_PATH=$ASSETS_PATH
echo "ROS2_5G_ERA_ASSETS_PATH set to $ASSETS_PATH"

# Directory with mmDetection from MMCV
MMDET_DIR=$(dirname "$ROOT_PATH")
MMDET_PATH="$MMDET_DIR/mmdetection/"
export ROS2_5G_ERA_MMDET_PATH=$MMDET_PATH
echo "ROS2_5G_ERA_MMDET_PATH set to $MMDET_PATH"

# Selection of model variant for mmDetection 
# - use "cv2_faces" to select the old opencv face detector
# - available mmDet models: "yolov3_mobilenet", "mask_rcnn_r50"
export NETAPP_MODEL_VARIANT="yolov3_mobilenet"
echo "NETAPP_MODEL_VARIANT set to $NETAPP_MODEL_VARIANT"

# Torch device ('cpu', 'cuda', 'cuda:0', etc.)
export NETAPP_TORCH_DEVICE="cpu"
echo "NETAPP_TORCH_DEVICE set to $NETAPP_TORCH_DEVICE"

