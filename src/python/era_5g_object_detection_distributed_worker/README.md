# era_5g_object_detection_distributed_worker

Implementation of distributed variant of object detector NetApp worker. 

## Reference implementation

The reference implementation of the NetApp worker is provided in the file era_5g_object_detection_distributed_interface/worker.py.

## Installation

The package could be installed via pip:

```bash
pip3 install -r requirements.txt
pip3 install .
```

## Examples

Set system environment variables:

```
# Set url of Celery message broker (e.g. RabbitMQ):
export CELERY_BROKER_URL="amqp://127.0.0.1:5672"

# Set url of Celery results backend (e.g. Redis)
export CELERY_RESULT_BACKEND="redis://127.0.0.1:6379"
```

```
# Run RabbitMQ message broker and Redis for storing results:
sudo docker run -p 5672:5672 rabbitmq:3.13
sudo docker run -p 6379:6379 redis:7.2
```

Run the worker using celery:
```bash
celery -A era_5g_object_detection_distributed_worker.worker worker -n test --concurrency 1 -P solo 
```
