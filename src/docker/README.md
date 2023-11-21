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

The reference implementation of standalone 5G-ERA Network Application docker image. The interface requires exposing 
port to the hosts to work properly. The 5896 port is required for HTTP communication with the interface. 
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
NOTE: obsolete, out of date, non-functional
Contains two Dockerfiles for the building of _interface__* and *worker* image. The interface requires exposing several ports to the hosts to work properly. The 5896 port is required for HTTP communication with the interface. *N* other ports are needed for the *GStreamer* communication. This reference implementation allows up to three concurrent GStreamer connections on ports 5001, 5002 and 5003. These ports are specified in the *interface.py* script and exposed in the Dockerfile.interface and the *run* command bellow.

How to build:
```bash
sudo docker build . -f docker/era_5g_object_detection_distributed/Dockerfile.interface -t but5gera/netapp_object_detection_distributed_interface:VERSION
sudo docker build . -f docker/era_5g_object_detection_distributed/Dockerfile.worker -t but5gera/netapp_object_detection_distributed_worker:0.1.0
```

How to run:
```bash
sudo docker run --rm -p 5896:5896 -p 5001-5003:5001-5003/udp but5gera/netapp_object_detection_distributed_interface:VERSION
sudo docker run --rm but5gera/netapp_object_detection_distributed_worker:0.1.0
```

