# era_5g_object_detection_distributed_interface

Implementation of distributed variant of object detector 5G-ERA Network Application interface. 
Requires the opencv to be compiled with the support of Gstreamer.

## Reference implementation

The reference implementation of the 5G-ERA Network Application interface is provided in the file 
era_5g_object_detection_distributed_interface/interface.py.

## Installation

The package could be installed via pip:

```bash
pip3 install -r requirements.txt
pip3 install .
```

## Examples

System environment variable that can be set, e.g.:

```
# port of the 5G-ERA Network Application's server (default is 5896)
export NETAPP_PORT=5897
```

```bash
python3 interface.py
```

An era_5g_object_detection_distributed_interface binary could be used to run the interface 
once the package is installed. 

## Classes

### ResultsReader (results_reader.py)

Thread-based class which search the jobs queue for finished tasks and
    publish results to the robot 

### TaskHandlerGstreamerRabbitmqRedis (task_handler_gstreamer_rabbitmq_redis.py)

Task handler which stores the retrieved images to the RabbitMQ broker.
