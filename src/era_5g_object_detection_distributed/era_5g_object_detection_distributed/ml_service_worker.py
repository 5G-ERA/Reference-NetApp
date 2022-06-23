# Celery worker

import logging
import os
import cv2
import numpy as np

from celery import Celery, Task
from celery.signals import worker_process_init

from typing import Dict  # for Python 3.9, just 'dict' will be fine

from mmdet.apis import init_detector, inference_detector
from mmdet.core import get_classes

from era_5g_helpers import get_path_to_assets, get_path_from_env
from era_5g_helpers.mmdet_utils import MODEL_VARIANTS, convert_mmdet_result


broker_url = os.environ.get("CELERY_BROKER_URL", "amqp://guest:guest@127.0.0.1:5672")
redis_url = os.environ.get("CELERY_RESULT_BACKEND", "redis://127.0.0.1/")
app = Celery('tasks', broker=broker_url, backend=redis_url, )
app.conf.task_serializer = 'pickle'
app.conf.result_serializer = 'pickle'
app.conf.accept_content = ['pickle']

path_to_assets = get_path_to_assets()

path_to_mmdet = get_path_from_env("ROS2_5G_ERA_MMDET_PATH")
model_variant = os.getenv("NETAPP_MODEL_VARIANT")
torch_device = os.getenv("NETAPP_TORCH_DEVICE", 'cpu') # 'cpu', 'cuda', 'cuda:0', etc.


class DetectorBase():
    def __init__(self):
         pass

    def inference(self, image):
        raise NotImplementedError


class MMDetector(DetectorBase):
    def __init__(self, model_variant, torch_device, logger=None):
        # Init detector
        print("Initializing mmDet detector.")
        config_file = os.path.join(path_to_mmdet, MODEL_VARIANTS[model_variant]['config_file'])
        checkpoint_file = os.path.join(path_to_mmdet, MODEL_VARIANTS[model_variant]['checkpoint_file'])
        self.model = init_detector(config_file, checkpoint_file, device=torch_device)

    def inference(self, image):    
        result = inference_detector(self.model, image)
        detections = convert_mmdet_result(result, merged_data=True)
        return detections


class FaceDetector(DetectorBase):
    def __init__(self):
        print("Initializing cv2 face detector.")
        self.detection_cascade = cv2.CascadeClassifier(os.path.join(path_to_assets, 'haarcascade_frontalface_default.xml'))

    def inference(self, image):
        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # gray = cv2.resize(gray,(640,360))
        # Detect the faces
        faces = self.detection_cascade.detectMultiScale(gray, 1.35, 4)

        detections = []
        
        for bbox in faces:
            # Transform from x,y,w,h to x1,y1,x2,y2 (bbox top-left bottom-right corners)
            bbox[2] += bbox[0] 
            bbox[3] += bbox[1]

            # Generate random class
            cls = 1  # np.random.randint(0, 80)
            cls_name = "face"

            # Generate random detection score
            score = np.random.random()
            det = bbox, score, cls, cls_name

            # Add to other detections for processed frame
            detections.append(det)
        
        return detections


detector_worker = None 

# Init
@worker_process_init.connect()
def worker_setup(**kwargs):
    global detector_worker
    if not model_variant or model_variant == "cv2_faces":
        detector_worker = FaceDetector()
    elif model_variant in MODEL_VARIANTS.keys():
        detector_worker = MMDetector(model_variant, torch_device)
    else:
        raise ValueError("Given model variant is currently not supported.")


@app.task()
def detector_task(data):
    metadata, image = data
    
    detections = detector_worker.inference(image)
    
    results = (metadata, detections)
    return results

