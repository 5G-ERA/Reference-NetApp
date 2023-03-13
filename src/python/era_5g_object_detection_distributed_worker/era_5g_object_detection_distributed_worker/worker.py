import os
import logging
from era_5g_object_detection_distributed_worker.worker_mmdet import MMDetectorWorker
from celery import Celery
from celery.signals import worker_process_init

from era_5g_object_detection_common.image_detector import ImageDetectorInitializationFailed


# RabbitMQ service address
broker_url = os.environ.get("CELERY_BROKER_URL")  # e.g. amqp://guest:guest@192.168.206.10:5672
# Redis service address
redis_url = os.environ.get("CELERY_RESULT_BACKEND")  # e.g. redis://192.168.206.11/

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
        detector_worker = MMDetectorWorker()
    except ImageDetectorInitializationFailed as ex:
        print(f"Failed to init the detector: {ex}")
        exit()


@app.task()
def detector_task(data):
    metadata, image = data
    # process the received image
    detections = detector_worker.process_image(image)
    results = (metadata, detections)
    return results
