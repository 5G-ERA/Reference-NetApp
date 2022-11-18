import os
from celery import Celery
from celery.signals import worker_process_init

from era_5g_object_detection_common.image_detector import ImageDetectorInitializationFailed

from era_5g_object_detection_common.fps_test_detector import FpsTestDetector
from era_5g_netapp_interface.common import get_logger
import logging

# RabbitMQ service address
broker_url = os.environ.get("CELERY_BROKER_URL") # e.g. amqp://guest:guest@192.168.206.10:5672
# Redis service address
redis_url = os.environ.get("CELERY_RESULT_BACKEND") # e.g. redis://192.168.206.11/

app = Celery('tasks', broker=broker_url, backend=redis_url, )
app.conf.task_serializer = 'pickle'
app.conf.result_serializer = 'pickle'
app.conf.accept_content = ['pickle']

detector_worker = None 

# Init
@worker_process_init.connect()
def worker_setup(**kwargs):
    global detector_worker
    try:
        # initialization of the detector
        detector_worker = FpsTestDetector(get_logger(logging.INFO), "Detector")
    except ImageDetectorInitializationFailed as ex:
        print(f"Failed to init the detector: {ex}")
        exit()


@app.task()
def detector_task(data):
    metadata, image = data
    # process the recieved image
    detections = detector_worker.process_image(image)
    results = (metadata, detections)
    return results