# Celery worker

import logging
import os
#import cv2
import numpy as np

from celery import Celery, Task
from celery.signals import worker_process_init

from typing import Dict  # for Python 3.9, just 'dict' will be fine

import rospy


broker_url = "amqp://" + rospy.get_param('~rabbitmq_URL') + ":5672"  
redis_url = "redis://" + rospy.get_param('~redis_URL') + ":6379"  

rospy.loginfo(broker_url)
rospy.loginfo(redis_url)
app = Celery('tasks', broker=broker_url, backend=redis_url)
app.conf.task_serializer = 'pickle'
app.conf.result_serializer = 'pickle'
app.conf.accept_content = ['pickle']
@app.task()
def detector_task(data):

    from mmdet.apis import init_detector, inference_detector
    from mmdet.core import get_classes

    from era_5g_helpers import get_path_to_assets, get_path_from_env
    from era_5g_helpers.mmdet_utils import MODEL_VARIANTS, convert_mmdet_result
    metadata, image = data
    
    detections = detector_worker.inference(image)
    
    results = (metadata, detections)
    return results



