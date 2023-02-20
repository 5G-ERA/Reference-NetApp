#!/bin/bash


ROOT_PATH=$(dirname $(realpath "$BASH_SOURCE"))
ASSETS_PATH="$ROOT_PATH/assets"


# IP address or hostname of the computer, where the NetworkApp is deployed
export NETAPP_ADDRESS="127.0.0.1"

# Port of the netapp's server
export NETAPP_PORT="5896"

# File with video test video
export TEST_VIDEO_FILE="$ASSETS_PATH/2017_01_02_001021_s.mp4"

# Basic opencv face detector
export NETAPP_FACE_DETECTOR_MODEL_FILE="$ASSETS_PATH/haarcascade_frontalface_default.xml"


# Directory with mmDetection
export NETAPP_MMDET_PATH="/srv/mmdetection/"
echo "NETAPP_MMDET_PATH set to $NETAPP_MMDET_PATH"

# Selection of model variant for mmDetection 
# - use "cv2_faces" to select the basic opencv face detector
# - available mmDet models: "yolov3_mobilenet", "mask_rcnn_r50"
export NETAPP_MODEL_VARIANT="yolov3_mobilenet"
echo "NETAPP_MODEL_VARIANT set to $NETAPP_MODEL_VARIANT"

# Torch device ('cpu', 'cuda', 'cuda:0', etc.)
export NETAPP_TORCH_DEVICE="cpu"
echo "NETAPP_TORCH_DEVICE set to $NETAPP_TORCH_DEVICE"


