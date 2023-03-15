import os
from celery import Celery
from celery.signals import worker_process_init


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
    pass


@app.task(name='era-5g-reference-netapp-distributed')
def detector_task(data):
    pass
