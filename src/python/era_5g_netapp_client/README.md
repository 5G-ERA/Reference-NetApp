# era_5g_netapp_client

A client application for 5G-ERA NetApps. Contains basic implementation of the client (client.py) and extended client for NetApps using the *gstreamer* for image streams transort. Besides, two classes for sending data using the h264 stream are presented.

## Reference implementation

The reference implementation of the NetApp client is provided in the file tests/test_client_gstreamer.py.

## Classes

### NetAppClient (client.py)

Basic implementation of the client. Allows to register with the NetApp and setup a websocket-based communication for reading of results.

### NetAppClientGstreamer (client_gstreamer.py)

Besides the basic functionality of the client, this class allows to setup a gstreamer-based channel for h264 stream transport.

### DataSenderGStreamer (data_sender_gstreamer.py)

Class which allows to construct the h264 stream using gstreamer and send it to the NetApp.

### DataSenderGStreamerFromSource (data_sender_gstreamer_from_source.py)

Class which utilizes gstreamer source (e.g. v4l2src for grabbing stream from webcamera) and pass it to the NetApp.
