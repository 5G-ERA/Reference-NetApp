# era_5g_netapp_client

A client application for 5G-ERA NetApps. Contains the basic implementation of the client (client.py) and extended client for NetApps using the *gstreamer* for image stream transport. Besides, two classes for sending data using the h264 stream are presented.

## Reference implementation

The reference implementation of the NetApp client is provided in the file tests/test_client_gstreamer.py.

## Installation

The package could be installed via pip:

```bash
pip3 install .
```

## Examples

Two example clients are provided to test the NetApp

### Using middleware (test/test_client_gstreamer.py)

The first example uses middleware to deploy the NetApp and to handle its life cycle. Therefore, the NetApp needs to be uploaded to the accessible docker repository and the NetApp needs to be registered within the middleware. Basic configuration needs to be provided:

- **server_ip**: IP address or hostname of the computer, where the middleware runs
- **user**: the GUID of the registered user
- **password**: the password of the registered user
- **task_id**: the GUID of the task that should be initialized

The user and password could be registered with the **/register** endpoint on the middleware (using Postman for example).

### Avoiding middleware (test/test_client_gstreamer_no_middleware.py)

For local testing, the middleware could be avoided, so the NetApp needs to be started manually. Basic configuration needs to be provided:

- **netapp_uri**: ip address or hostname of the computer, where the NetApp is deployed
- **netapp_port**: port of the NetApp's server (default is 5896)

## Classes

### NetAppClient (client.py)

Basic implementation of the client. Allows to register with the NetApp and setup a websocket-based communication for reading of results.

### NetAppClientGstreamer (client_gstreamer.py)

Besides the basic functionality of the client, this class allows to setup a gstreamer-based channel for h264 stream transport.

### DataSenderGStreamer (data_sender_gstreamer.py)

Class which allows to construct the h264 stream using gstreamer and send it to the NetApp.

### DataSenderGStreamerFromSource (data_sender_gstreamer_from_source.py)

Class which utilizes gstreamer source (e.g. v4l2src for grabbing stream from webcamera) and pass it to the NetApp.
