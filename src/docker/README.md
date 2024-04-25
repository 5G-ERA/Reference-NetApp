# Dockerfiles and related stuff

Each folder contains Dockerfiles and running scripts for each reference image. 

## Folders

### era_5g_netapp_base_mmcv

Base docker image for 5G-ERA Network Applications based on MMCV detectors. Based on python:3.8-slim image with torch, 
mmcv and mmdetection installed. Two Dockerfiles are provided, one for the CPU-only variant and one which includes 
the CUDA for GPU processing.

How to build (CPU-only):
```bash
sudo docker build . -f docker/era_5g_netapp_base_mmcv/cpu.Dockerfile -t but5gera/netapp_base_mmcv_cpu:VERSION
```

How to build (CUDA):
```bash
sudo docker build . -f docker/era_5g_netapp_base_mmcv/gpu.Dockerfile -t but5gera/netapp_base_mmcv_gpu:VERSION
```

### era_5g_object_detection_standalone

The reference implementation of standalone 5G-ERA Network Application docker image. The interface requires exposing a 
port to the hosts to work properly. The 5896 port is required for communication with the interface. 
Two Dockerfiles are provided, one for the CPU-only variant and one which includes the CUDA for GPU processing.

How to build (CPU-only):
```bash
sudo docker build . -f docker/era_5g_object_detection_standalone/cpu.Dockerfile -t but5gera/netapp_object_detection_standalone_cpu:VERSION
```

How to build (CUDA):
```bash
sudo docker build . -f docker/era_5g_object_detection_standalone/gpu.Dockerfile -t but5gera/netapp_object_detection_standalone_gpu:VERSION
```

How to run:
```bash
sudo docker run --rm -p 5896:5896 -d --gpus all but5gera/netapp_object_detection_standalone_gpu:VERSION
```

### ros2_object_detection
Based on netapp_base_mmcv_gpu. ROS 2 object detection using CUDA and MMDetector.

How to build (CUDA):
```bash
sudo docker build . -f docker/ros2_object_detection/Dockerfile -t but5gera/ros2_object_detection:VERSION
```

How to run (set correct INPUT_TOPIC):
```bash
sudo docker run --network=host --ipc=host --pid=host --rm -d --gpus all -e INPUT_TOPIC=/image_raw -e OUTPUT_TOPIC=/results but5gera/ros2_object_detection:VERSION
```

Publish image on /image_raw topic, e.g.:
```bash
ros2 run image_publisher image_publisher_node video.mp4
```

### ros_object_detection
Based on netapp_base_mmcv_gpu. ROS 1 object detection using CUDA and MMDetector.

How to build (CUDA):
```bash
sudo docker build . -f docker/ros_object_detection/Dockerfile -t but5gera/ros_object_detection:VERSION
```

How to run (set correct INPUT_TOPIC):
```bash
sudo docker run --network=host --ipc=host --pid=host --rm -d --gpus all -e INPUT_TOPIC=/image_raw -e OUTPUT_TOPIC=/results but5gera/ros_object_detection:VERSION
```

Publish image on INPUT_TOPIC topic, e.g.:
```bash
rosrun image_publisher image_publisher video.mp4
```

### era_5g_object_detection_distributed
Dockerfiles are provided for building *interface* and *worker* images. The interface requires exposing a 
port to the hosts to work properly (default port is 5896).

How to build:
```bash
sudo docker build . -f docker/era_5g_object_detection_distributed/Dockerfile.interface -t but5gera/netapp_object_detection_distributed_interface:VERSION
sudo docker build . -f docker/era_5g_object_detection_distributed/Dockerfile.worker_gpu -t but5gera/netapp_object_detection_distributed_worker_gpu:VERSION
sudo docker build . -f docker/era_5g_object_detection_distributed/Dockerfile.worker -t but5gera/netapp_object_detection_distributed_worker:VERSION
```

How to run:

Run RabbitMQ message broker and Redis for storing results:
```bash
sudo docker run -p 5672:5672 rabbitmq:3.13
sudo docker run -p 6379:6379 redis:7.2
```

Run worker (either CPU or GPU version) and Network Application interface.

CPU worker:
```bash
sudo docker run --network=host -e CELERY_RESULT_BACKEND="redis://127.0.0.1:6379" but5gera/netapp_object_detection_distributed_worker:0.3.0
```

GPU worker:
```bash
sudo docker run --gpus=all --network=host -e CELERY_RESULT_BACKEND="redis://127.0.0.1:6379" but5gera/netapp_object_detection_distributed_worker_gpu:0.3.0
```

Run interface:
```bash
sudo docker run -p 5896:5896 --network=host -e CELERY_RESULT_BACKEND="redis://127.0.0.1:6379" but5gera/netapp_object_detection_distributed_interface:0.3.0

```

Note: Environment varibale CELERY_BROKER_URL can also be set when RabbitMQ is not running on localhost.
