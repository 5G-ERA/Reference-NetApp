rosdep install -i --from-path src --rosdistro galactic -y
apt-get install -y python3-pip
pip install requests
#sudo -H pip install -U git+https://github.com/eric-wieser/ros_numpy.git #ROS1!!!


# CPU version
pip install torch torchvision torchaudio --extra-index-url https://download.pytorch.org/whl/cpu
pip install mmcv-full==1.5.2


# Get mmDet and download model weights

source ./set_environment.sh

# Tested with version 2.25.0
git clone -b v2.25.0 https://github.com/open-mmlab/mmdetection.git "$MMDET_PATH"
cd "$MMDET_PATH"
pip install .

apt-get install -y wget
wget -c https://download.openmmlab.com/mmdetection/v2.0/yolo/yolov3_mobilenetv2_320_300e_coco/yolov3_mobilenetv2_320_300e_coco_20210719_215349-d18dff72.pth -O "$MMDET_PATH/configs/yolo/yolov3_mobilenetv2_320_300e_coco_20210719_215349-d18dff72.pth"

