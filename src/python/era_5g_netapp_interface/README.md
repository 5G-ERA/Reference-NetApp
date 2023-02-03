# era_5g_netapp_interface

Support classes for 5G-ERA NetApps.

## Installation

The package could be installed via pip:

```bash
pip3 install .
```

## Classes

### ThreadBase (common.py)

 Base Thread class which provides handy methods.

### TaskHandler (task_handler.py)

Abstract class. Thread-based task handler which takes care of recieving data from the NetApp client and passing them to the NetApp worker.

### TaskHandlerGstreamer (task_handler_gstreamer.py)

Abstract class. Task handler which takes care of reading the data from Gstreamer pipeline with defined parameters. It needs to be inherited to implement the store_image method.

### TaskHandlerInternalQ (task_handler_internal_q.py)

Task handler which takes care of passing the data to the python internal queue for future processing. It could either be inherited to implement the _run method and read the data from any source or used directly and call the store_image method externaly.

### TaskHandlerGstreamerInternalQ (task_handler_gstreamer_internal_q.py)

Task handler which combines the Gstreamer functionality of data retrival with usage of python internal queues for passing the data to the worker object.
