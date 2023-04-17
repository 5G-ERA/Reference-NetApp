FROM python:3.8-slim
#inspired by https://github.com/juri117/python-opencv-gstreamer-docker-container

RUN DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends tzdata

RUN apt-get update \
    && apt-get install -y \
    git \
    python3-pip \
    build-essential \
    cmake \
    gcc \
    ffmpeg \
    libgstreamer1.0-0 \
    gstreamer1.0-plugins-base \
    gstreamer1.0-libav \
    gstreamer1.0-tools \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev

RUN cd /root/ \
    && git clone --recursive https://github.com/opencv/opencv-python.git \
    && cd opencv-python/opencv \
    && git checkout 4.6.0 \
    && cd ../opencv_contrib \
    && git checkout 4.6.0

RUN pip3 install --upgrade pip setuptools wheel

RUN cd /root/opencv-python/ \
    && export CMAKE_ARGS="-DWITH_GSTREAMER=ON -DCMAKE_BUILD_TYPE=RELEASE" \
    && pip3 wheel . --verbose

FROM python:3.8-slim

RUN mkdir -p /root/opencv

COPY --from=0 /root/opencv-python/*.whl /root/opencv/

RUN apt-get update \
    && apt-get install -y \
    libgstreamer1.0-0 \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-libav \
    gstreamer1.0-tools \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev

RUN cd /root/opencv \
    && pip3 install *.whl