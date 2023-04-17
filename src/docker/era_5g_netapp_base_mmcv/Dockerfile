FROM python:3.8-slim

RUN apt-get update \
    && apt-get install -y python3-pip git wget

# CPU version
RUN pip3 install torch===1.13.1+cpu torchvision===0.14.1+cpu torchaudio===0.13.1+cpu --extra-index-url https://download.pytorch.org/whl/cpu

RUN pip3 install mmcv-full==1.5.2

RUN cd /root/ \
    && git clone -b v2.25.0 https://github.com/open-mmlab/mmdetection.git

RUN cd /root/mmdetection \
    && pip3 install .

ENV NETAPP_MMDET_PATH=/root/mmdetection

#RUN mkdir -p /root/mmdetection/configs/yolo/ \
#    && cd /root/mmdetection/configs/yolo/ \

#&& wget -c https://download.openmmlab.com/mmdetection/v2.0/yolo/yolov3_mobilenetv2_320_300e_coco/yolov3_mobilenetv2_320_300e_coco_20210719_215349-d18dff72.pth


