# Installation

```bash
git clone git@github.com:5G-ERA/Reference-NetApp.git

cd Reference-NetApp

python3 -m venv env
source env/bin/activate
```

To properly install mmcv, it is first necessary to install torch with CUDA and then mmcv according to the 
instructions, e.g. [here](era_5g_object_detection_common/README.md) 

```bash
cd Reference-NetApp/src/python/era_5g_object_detection_common
pip3 install -r requirements.txt
pip3 install -e .

cd ../era_5g_object_detection_standalone
pip3 install -r requirements.txt
pip3 install -e .
```
