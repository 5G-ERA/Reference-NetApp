# Dockerfiles and related stuff

Each folder contains Dockerfiles and running scripts for each reference image. 

## Folders

### era_5g_netapp_base_gstreamer

Base docker image for all NetApps which want to use GStreamer in python. Based on python:3.8-slim image with opencv-python compiled with GStreamer.

How to build:
```bash
sudo docker build . -f docker/era_5g_netapp_base_gstreamer/Dockerfile -t but5gera/netapp_base_gstreamer:VERSION
```

### era_5g_netapp_base_mmcv

Base docker image for NetApps based on MMCV detectors. Based on python:3.8-slim image with torch, mmcv-full and mmdetection installed. Two Dockerfiles are provided, one for the CPU-only variant and one which includes the CUDA for GPU processing.

How to build (CPU-only):
```bash
sudo docker build . -f docker/era_5g_netapp_base_mmcv/Dockerfile -t but5gera/netapp_base_mmcv_cpu:VERSION
```

How to build (CUDA):
```bash
sudo docker build . -f docker/era_5g_netapp_base_mmcv/Dockerfile.gpu -t but5gera/netapp_base_mmcv_gpu:VERSION
```

### era_5g_object_detection_standalone

The reference implementation of standalone NetApp docker image. The interface requires exposing several ports to the hosts to work properly. The 5896/udp port is required for HTTP communication with the interface. *N* other ports are needed for the *GStreamer* communication. This reference implementation allows up to three concurrent GStreamer connections on ports 5001, 5002 and 5003. These ports are specified in the *interface.py* script and exposed in the Dockerfile.interface and the *run* command below. Two Dockerfiles are provided, one for the CPU-only variant and one which includes the CUDA for GPU processing.

How to build (CPU-only):
```bash
sudo docker build . -f docker/era_5g_object_detection_standalone/Dockerfile -t but5gera/netapp_object_detection_standalone_cpu:VERSION
```

How to build (CUDA):
```bash
sudo docker build . -f docker/era_5g_object_detection_standalone/Dockerfile.gpu -t but5gera/netapp_object_detection_standalone_gpu:VERSION
```

How to run:
```bash
sudo docker run --rm -p 5896:5896/udp -p 5001-5003:5001-5003/udp but5gera/netapp_object_detection_standalone_cpu:VERSION
```

### era_5g_object_detection_distributed

Contains two Dockerfiles for the building of _interface__* and *worker* image. The interface requires exposing several ports to the hosts to work properly. The 5896/udp port is required for HTTP communication with the interface. *N* other ports are needed for the *GStreamer* communication. This reference implementation allows up to three concurrent GStreamer connections on ports 5001, 5002 and 5003. These ports are specified in the *interface.py* script and exposed in the Dockerfile.interface and the *run* command bellow.

How to build:
```bash
sudo docker build . -f docker/era_5g_object_detection_distributed/Dockerfile.interface -t but5gera/netapp_object_detection_distributed_interface:VERSION
sudo docker build . -f docker/era_5g_object_detection_distributed/Dockerfile.worker -t but5gera/netapp_object_detection_distributed_worker:0.1.0
```

How to run:
```bash
sudo docker run --rm -p 5896:5896/udp -p 5001-5003:5001-5003/udp but5gera/netapp_object_detection_distributed_interface:VERSION
sudo docker run --rm but5gera/netapp_object_detection_distributed_worker:0.1.0
```

