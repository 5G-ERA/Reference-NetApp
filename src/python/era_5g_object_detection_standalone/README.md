# era_5g_object_detection_standalone

Implementation of standalone variant of object detector NetApp. Requires the opencv to be compiled with the support of Gstreamer. The range of ports available for the Gstreamer transport.

## Reference implementation

The reference implementation of the NetApp client is provided in the file era_5g_object_detection_standalone/interface.py. 

## Installation

The package could be installed via pip:

```bash
pip3 install -r requirements.txt
pip3 install .
```

An era_5g_object_detection_standalone binary could be used to run the NetApp once the package is installed. 

## Examples

System environment variable that can be set, e.g.:

```
# port of the NetApp's server (default is 5896)
export NETAPP_PORT=5897
```

Arguments:

 - --ports
 
   Specify the range of ports available for gstreamer connections. Format port_start:port_end. 
   Default is 5001:5003.
 
 - --detector

   Select detector. Available options are opencv, mmdetection, fps. Default is fps.

```bash
python3 interface.py --detector mmdetection
```

## Classes

### Worker (worker.py)

Worker object for image processing in standalone variant. Reads data from passed queue, performs detection and returns results usingthe flask app. Needs to be inherited to implement the process_image method.

### FaceDetectorWorker (worker_face.py)

Worker object for the basic face detector based on OpenCV Haar-cascade Detection. 

### FpsDetectorWorker (worker_fps.py)

Worker object for the debug detector which returns the framerate of recieved stream.

### MMDetectorWorker (worker_mmdet.py)

Worker object for the universal detector based on MMDET package.
