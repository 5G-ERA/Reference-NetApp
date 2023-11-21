FROM but5gera/netapp_base_mmcv_cpu:0.2.0

ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=Europe/Prague

RUN pip install --upgrade pip

COPY python/era_5g_object_detection_common /root/era_5g_object_detection_common

RUN cd /root/era_5g_object_detection_common \
    && pip install -r requirements.txt \
    && pip install .

COPY python/era_5g_object_detection_standalone /root/era_5g_object_detection_standalone

RUN cd /root/era_5g_object_detection_standalone \
    && pip install -r requirements.txt \
    && pip install .

COPY docker/era_5g_object_detection_standalone/start.sh /root/start.sh

ENTRYPOINT ["/root/start.sh"]

RUN chmod +x /root/start.sh

ENV NETAPP_MODEL_VARIANT=yolov3_mobilenet
ENV NETAPP_TORCH_DEVICE=cpu
    
EXPOSE 5896