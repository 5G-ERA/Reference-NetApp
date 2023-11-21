# era_5g_object_detection_common

Support classes for 5G-ERA object detectors 5G-ERA Network Applications.

## Installation

The package could be installed via pip:

```bash
pip3 install -r requirements.txt
pip3 install .
```

## Classes

### ImageDetector (image_detector.py)

The base class for 5G-ERA Network Applications based on image processing. Provides abstract methods for processing images and publishing results. It is based on Threads.

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

```
pip3 install torch torchvision torchaudio

pip install -U openmim
mim install mmcv

git clone https://github.com/open-mmlab/mmdetection.git
cd mmdetection
pip install -v -e .

cd configs/yolo/
mim download mmdet --config yolov3_mobilenetv2_8xb24-320-300e_coco  --dest .
mim download mmdet --config yolov3_d53_320_273e_coco  --dest .
mim download mmdet --config yolov3_d53_mstrain-416_273e_coco  --dest .

cd ../yolox/
mim download mmdet --config yolox_tiny_8x8_300e_coco  --dest .

cd ../mask_rcnn/
mim download mmdet --config mask-rcnn_r50-caffe_fpn_ms-poly-3x_coco  --dest .

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