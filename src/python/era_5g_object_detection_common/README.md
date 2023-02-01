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

### FpsTestDetector (fps_test_detector.py)

Debug detector which returns the framerate of recieved stream.

### MMDetector (mm_detector.py)

Universal detector based on MMDET package.
