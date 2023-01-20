#!/bin/bash

cd
celery -A era_5g_object_detection_distributed_worker.worker worker -n test --concurrency 1 -P solo 