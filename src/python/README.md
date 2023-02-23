# Installation

```bash
git clone git@github.com:5G-ERA/Reference-NetApp.git

cd Reference-NetApp

python3 -m venv env
source env/bin/activate

cd
cp -r /srv/opencv ~
pip3 install ~/opencv/*.whl

# Install client from era-5g-client repository

# Install NetApp interface from era-5g-interface repository

cd Reference-NetApp/src/python/era_5g_object_detection_common
pip3 install -r requirement.txt
pip3 install -e .

cd ../era_5g_object_detection_distributed_interface
pip3 install -r requirement.txt
pip3 install -e .

cd ../era_5g_object_detection_distributed_worker
pip3 install -r requirement.txt
pip3 install -e .

cd ../era_5g_object_detection_standalone
pip3 install -r requirement.txt
pip3 install -e .
```
