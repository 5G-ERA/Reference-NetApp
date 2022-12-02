# era_5g_object_detection_distributed_worker

Implementation of distributed variant of object detector NetApp worker. 

## Reference implementation

The reference implementation of the NetApp worker is provided in the file era_5g_object_detection_distributed_interface/worker.py.

## Instalation

The package could be installed via pip:

```bash
pip3 install -r requirement.txt
pip3 install .
```

Run the worker using celery:
```bash
celery -A era_5g_object_detection_distributed_worker.worker worker -n test --concurrency 1 -P solo 
```
