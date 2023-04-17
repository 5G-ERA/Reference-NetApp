FROM but5gera/netapp_base_gstreamer:0.1.1

FROM but5gera/netapp_base_mmcv_cpu:0.1.0

RUN mkdir -p /root/opencv

COPY --from=0 /root/opencv/*.whl /root/opencv/

ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=Europe/Prague

RUN apt-get update \
    && apt-get install -y \
    libgstreamer1.0-0 \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    gstreamer1.0-tools \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev

RUN cd /root/opencv \
    && pip3 install *.whl

RUN wget -c https://download.openmmlab.com/mmdetection/v2.0/yolo/yolov3_mobilenetv2_320_300e_coco/yolov3_mobilenetv2_320_300e_coco_20210719_215349-d18dff72.pth -O "/root/mmdetection/configs/yolo/yolov3_mobilenetv2_320_300e_coco_20210719_215349-d18dff72.pth"




ENTRYPOINT ["/root/start.sh"]

COPY python/era_5g_object_detection_common /root/era_5g_object_detection_common

RUN cd /root/era_5g_object_detection_common \
    && pip3 install -r requirement.txt \
    && pip3 install . 

COPY python/era_5g_object_detection_standalone /root/era_5g_object_detection_standalone

RUN cd /root/era_5g_object_detection_standalone \
    && pip3 install -r requirement.txt \
    && pip3 install . 

COPY docker/era_5g_object_detection_standalone/start.sh /root/start.sh

RUN chmod +x /root/start.sh

ENV NETAPP_MODEL_VARIANT=yolov3_mobilenet
ENV NETAPP_TORCH_DEVICE=cpu
    
EXPOSE 5896
EXPOSE 5001 5002 5003
