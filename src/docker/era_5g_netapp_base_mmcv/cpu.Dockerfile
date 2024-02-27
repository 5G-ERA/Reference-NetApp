FROM python:3.8-slim

RUN apt-get update \
    && apt-get install -y tzdata python3-pip python-is-python3 git ffmpeg

RUN pip install --upgrade pip

RUN pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu

RUN pip install -U openmim

RUN mim install mmcv

RUN cd /root/ && git clone https://github.com/open-mmlab/mmdetection.git

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


