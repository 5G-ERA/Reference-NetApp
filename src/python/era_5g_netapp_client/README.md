# era_5g_netapp_client

A client application for 5G-ERA NetApps. Contains the basic implementation of the client (client.py) 
and extended client for NetApps using the *gstreamer* for image stream transport. Besides, 
two classes for sending data using the h264 stream are presented.

## Reference implementation

The reference implementations of the NetApp client is provided in the files tests/test_client_*.py.

## Installation

The package could be installed via pip:

```bash
pip3 install -r requirement.txt
pip3 install .
```

## Examples

Three example clients are provided to test the NetApp.

System environment variables can be set, e.g.:
```
# Use video file
export FROM_SOURCE=False
export TEST_VIDEO_FILE=../../../../assets/2017_01_02_001021_s.mp4
```
or
```
# Use webcam
export FROM_SOURCE=True
```

### Using middleware and gstreamer (test/test_client_gstreamer.py)

The first example uses middleware to deploy the NetApp and to handle its life cycle. 
Therefore, the NetApp needs to be uploaded to the accessible docker repository 
and the NetApp needs to be registered within the middleware. Basic configuration needs to be provided:

System environment variables that should be set:

```
# ip address or hostname of the computer, where the middleware runs
export MIDDLEWARE_ADDRESS=middleware_address
# the GUID of the registered middleware user
export MIDDLEWARE_USER=middleware_user
the password of the registered middleware user
export MIDDLEWARE_PASSWORD=middleware_password
the middleware GUID of the task that should be initialized
export MIDDLEWARE_TASK_ID=middleware_task_id
```

The user and password could be registered with the **/register** endpoint on the middleware 
(using Postman for example).

```bash
python3 test_client_gstreamer.py
```

### Avoiding middleware, using gstreamer transport (test/test_client_gstreamer_no_middleware.py)

For local testing, the middleware could be avoided, so the NetApp needs to be started manually. 
Basic configuration needs to be provided:

System environment variables that can be set, e.g.:

```
# ip address or hostname of the computer, where the NetApp is deployed
export NETAPP_ADDRESS=127.0.0.1
# port of the NetApp's server (default is 5896)
export NETAPP_PORT=5897
```

```bash
python3 test_client_gstreamer_no_middleware.py
```

### Avoiding middleware, using http transport (test/test_client_http.py)

The config is the same as in the previous case, but instead of GStreamer, the images are sent to 
the REST endpoint of the NetApp, so the timestamps are preserved and the results could be drawn 
into the images and shown. 

```bash
python3 test_client_http.py
```

## Classes

### NetAppClient (client.py)

Basic implementation of the client. Allows to register with the NetApp and set the websocket-based 
communication for reading of results.

### NetAppClientGstreamer (client_gstreamer.py)

Besides the basic functionality of the client, this class allows to set the gstreamer-based channel 
for h264 stream transport.

### DataSenderGStreamer (data_sender_gstreamer.py)

Class which allows to construct the h264 stream using gstreamer and send it to the NetApp.

### DataSenderGStreamerFromFile (data_sender_gstreamer_from_file.py)

Class which utilizes video file as gstreamer source and pass it to the NetApp.

### DataSenderGStreamerFromSource (data_sender_gstreamer_from_source.py)

Class which utilizes gstreamer source (e.g. v4l2src for grabbing stream from webcam) and pass it to 
the NetApp.
