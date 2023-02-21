# era_5g_object_detection_common

Support classes for 5G-ERA object detectors NetApps.

## Installation

The package could be installed via pip:

```bash
pip3 install -r requirement.txt
pip3 install .
```

## Classes

### ThreadBase (common.py)

Base Thread class which provides handy methods.

### ImageDetector (image_detector.py)

The base class for NetApps based on image processing. Provides abstract methods for processing images and publishing results. It is based on Threads.

### FaceDetector (face_detector.py)

Basic face detector based on OpenCV Haar-cascade Detection. 

System environment variables that must be set, e.g.:

```
NETAPP_FACE_DETECTOR_MODEL_FILE=../../../../assets/haarcascade_frontalface_default.xml
```

### FpsTestDetector (fps_test_detector.py)

Debug detector which returns the framerate of received stream.

### MMDetector (mm_detector.py)

Universal detector based on MMDET package.

Requirements, install steps:

```bash
pip3 install torch torchvision torchaudio
pip3 install mmcv-full
git clone https://github.com/open-mmlab/mmdetection.git
cd mmdetection
pip install -v -e .
pip install mmdet
cd configs/yolo/
wget -c https://download.openmmlab.com/mmdetection/v2.0/yolo/yolov3_mobilenetv2_320_300e_coco/yolov3_mobilenetv2_320_300e_coco_20210719_215349-d18dff72.pth
```

System environment variables that must be set, e.g.:

```
# path to mmdetection root dir
export NETAPP_MMDET_PATH=../../../../../mmdetection
# model variant
export NETAPP_MODEL_VARIANT=yolov3_mobilenet
```

System environment variables that can be set, e.g.:

```
# Torch device ('cpu', 'cuda', 'cuda:0', etc.), default: 'cpu' 
export NETAPP_TORCH_DEVICE=cpu
```