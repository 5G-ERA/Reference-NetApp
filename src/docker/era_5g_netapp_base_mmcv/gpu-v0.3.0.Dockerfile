FROM nvidia/cuda:12.2.0-base-ubuntu22.04
# CUDA image is needed for kubernetes

ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=Europe/Prague

RUN apt-get update \
    && apt-get install -y tzdata python3-pip python-is-python3 git ffmpeg

RUN pip install --upgrade pip

RUN pip install torch===2.1.2+cu121 torchvision===0.16.2+cu121 torchaudio===2.1.2+cu121 --extra-index-url https://download.pytorch.org/whl/cu121

RUN pip install -U openmim==0.3.9

RUN mim install mmcv==2.1.0

RUN cd /root/ && git clone -b v3.2.0 https://github.com/open-mmlab/mmdetection.git

RUN cd /root/mmdetection && pip3 install .

ENV NETAPP_MMDET_PATH=/root/mmdetection

RUN mkdir -p /root/mmdetection/configs/yolo/ \
    && cd /root/mmdetection/configs/yolo/ \
    && mim download mmdet --config yolov3_mobilenetv2_8xb24-320-300e_coco  --dest . \
    && mim download mmdet --config yolov3_d53_320_273e_coco  --dest . \
    && mim download mmdet --config yolov3_d53_mstrain-416_273e_coco  --dest .

RUN mkdir -p /root/mmdetection/configs/yolox/ \
    && cd /root/mmdetection/configs/yolox/ \
    && mim download mmdet --config yolox_tiny_8x8_300e_coco  --dest .

RUN mkdir -p /root/mmdetection/configs/mask_rcnn/ \
    && cd /root/mmdetection/configs/mask_rcnn/ \
    && mim download mmdet --config mask-rcnn_r50-caffe_fpn_ms-poly-3x_coco  --dest .


