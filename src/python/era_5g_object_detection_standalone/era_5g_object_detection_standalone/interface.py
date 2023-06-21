import base64

import argparse
import binascii
import os
import numpy as np
import logging
import time

import socketio
from queue import Queue

from typing import Dict
from flask import Flask

from era_5g_object_detection_common.image_detector import ImageDetectorInitializationFailed
from era_5g_interface.task_handler_internal_q import TaskHandlerInternalQ, TaskHandler
from era_5g_object_detection_common.image_detector import ImageDetector
from era_5g_object_detection_standalone.worker_mmdet import MMDetectorWorker
from era_5g_object_detection_standalone.worker_fps import FpsDetectorWorker
from era_5g_object_detection_standalone.worker_face import FaceDetectorWorker

# port of the netapp's server
NETAPP_PORT = os.getenv("NETAPP_PORT", 5896)
# input queue size
NETAPP_INPUT_QUEUE = int(os.getenv("NETAPP_INPUT_QUEUE", 1))

#image_queue = Queue(NETAPP_INPUT_QUEUE)
    
# the max_http_buffer_size parameter defines the max size of the message to be passed
sio = socketio.Server(async_mode='threading', max_http_buffer_size=5*(1024**2))
app = Flask(__name__)
app.wsgi_app = socketio.WSGIApp(sio, app.wsgi_app)

# the image detector to be used
detector_threads: Dict[str, ImageDetector] = {}

tasks: Dict[str, TaskHandler] = {}

DetectorWorker: type = None

class ArgFormatError(Exception):
    pass

def get_sid_of_namespace(eio_sid, namespace):
    return sio.manager.sid_from_eio_sid(eio_sid, namespace)

def get_results_sid(eio_sid):
    return sio.manager.sid_from_eio_sid(eio_sid, "/results")

@sio.on('connect', namespace='/data')
def connect_data(sid, environ):
    """_summary_
    Creates a websocket connection to the client for passing the data.

    Raises:
        ConnectionRefusedError: Raised when attempt for connection were made
            without registering first.
    """
    print(f"Connected data. Session id: {sio.manager.eio_sid_from_sid(sid, '/data')}, namespace_id: {sid}")
    image_queue = Queue(NETAPP_INPUT_QUEUE)
    task = TaskHandlerInternalQ(sio.manager.eio_sid_from_sid(sid, '/data'), image_queue, daemon=True)
    eio_sid = sio.manager.eio_sid_from_sid(sid, '/data')
    tasks[eio_sid] = task
    detector = DetectorWorker(image_queue, sio, name=f"Detector {eio_sid}")
    detector_threads[eio_sid] = detector
    detector.start()
    sio.send("you are connected", namespace='/data', to=sid)

@sio.on('connect', namespace='/control')
def connect_control(sid, environ):
    """_summary_
    Creates a websocket connection to the client for passing control commands.

    Raises:
        ConnectionRefusedError: Raised when attempt for connection were made
            without registering first.
    """

    print(f"Connected control. Session id: {sio.manager.eio_sid_from_sid(sid, '/data')}, namespace_id: {sid}")
    sio.send("you are connected", namespace='/control', to=sid)

@sio.on('connect', namespace='/results')
def connect_results(sid, environ):
    """
    Creates a websocket connection to the client for passing the results.

    Raises:
        ConnectionRefusedError: Raised when attempt for connection were made
            without registering first.
    """

    print(f"Connected results. Session id: {sio.manager.eio_sid_from_sid(sid, '/data')}, namespace_id: {sid}")
    sio.send("You are connected", namespace='/results', to=sid)

@sio.on('image', namespace='/data')
def image_callback_websocket(sid, data: dict):
    """
    Allows to receive jpg-encoded image using the websocket transport

    Args:
        data (dict): A base64 encoded image frame and (optionally) related timestamp in format:
            {'frame': 'base64data', 'timestamp': 'int'}

    Raises:
        ConnectionRefusedError: Raised when attempt for connection were made
            without registering first or frame was not passed in correct format.
    """
    recv_timestamp = time.time_ns()
    
    if 'timestamp' in data:
        timestamp = data['timestamp']
    else:
        logging.debug("Timestamp not set, setting default value")
        timestamp = 0

    eio_sid = sio.manager.eio_sid_from_sid(sid, "/data")

    if eio_sid not in tasks:
        logging.error(f"Non-registered client tried to send data")
        sio.emit(
            "image_error",
            {"timestamp": timestamp,
             "error": "Not connected"},
                namespace='/data',
                to=sid
            )
        return
      
    if "frame" not in data:
        logging.error(f"Data does not contain frame.")
        sio.emit(
            "image_error",
            {"timestamp": timestamp,
             "error": f"Data does not contain frame."},
                namespace='/data',
                to=sid
            )
        return
    
    task = tasks[eio_sid]  
    try:
        frame = base64.b64decode(data["frame"])
        
    except (ValueError, binascii.Error) as error:
        logging.error(f"Failed to decode frame data: {error}")
        sio.emit(
            "image_error",
            {"timestamp": timestamp,
             "error": f"Failed to decode frame data: {error}"},
            namespace='/data',
            to=sid
            )
        return
    
    task.store_image(
            {"sid": eio_sid,
            "timestamp": data["timestamp"],
            "recv_timestamp": recv_timestamp,
            "websocket_id": get_results_sid(sio.manager.eio_sid_from_sid(sid, "/data")),
            "decoded": False},
            np.frombuffer(frame, dtype=np.uint8))            
    

@sio.on('json', namespace='/data')
def json_callback_websocket(sid, data):
    """
    Allows to receive general json data using the websocket transport

    Args:
        data (dict): NetApp-specific json data

    Raises:
        ConnectionRefusedError: Raised when attempt for connection were made
            without registering first.
    """
    print (data)
    
    logging.debug(f"client with task id: {sio.manager.eio_sid_from_sid(sid, '/data')} sent data {data}")
    


@sio.on('disconnect', namespace='/results')
def disconnect_results(sid):
    print(f"Client disconnected from /results namespace: session id: {sid}")


@sio.on('disconnect', namespace='/data')
def disconnect_data(sid):
    eio_sid = sio.manager.eio_sid_from_sid(sid, "/data")
    task = tasks.pop(eio_sid)
    detector = detector_threads.pop(eio_sid)
    task.stop()
    detector.stop()
    print(f"Client disconnected from /data namespace: session id: {sid}")

@sio.on('disconnect', namespace='/results')
def disconnect_data(sid):
    print(f"Client disconnected from /results namespace: session id: {sid}")


def main():
    parser = argparse.ArgumentParser(description='Standalone variant of object detection NetApp')
    # TODO: in future versions, the ports should be specified as list instead of range, preferably
    #       using env variable, for compatibility with middleware
    parser.add_argument(
        '--detector',
        default="mmdetection",
        help="Select detector. Available options are opencv, mmdetection, fps. Default is fps."
        )
    
    args = parser.parse_args()
    global DetectorWorker    
    
    logging.getLogger().setLevel(logging.DEBUG)
    # Creates detector and runs it as thread, listening to image_queue
    try:
        if args.detector == "fps":
            DetectorWorker = FpsDetectorWorker
        elif args.detector == "mmdetection":
            DetectorWorker = MMDetectorWorker
        elif args.detector == "opencv":
            DetectorWorker = FaceDetectorWorker
        else:
            raise ImageDetectorInitializationFailed(
                "Invalid detector selected. Available options are opencv, mmdetection, fps."
            )
    
    except ImageDetectorInitializationFailed as ex:
        print(ex)
        exit()

    # Estimate new queue size based on maximum latency
    """avg_latency = latency_measurements.get_avg_latency() # obtained from detector init warmup
    if avg_latency != 0:  # warmup can be skipped
        new_queue_size = int(max_latency / avg_latency)
        image_queue = Queue(new_queue_size)
        detector_thread.input_queue = image_queue"""
    
    logging.info(f"The size of the queue set to: {NETAPP_INPUT_QUEUE}")

    # runs the flask server
    # allow_unsafe_werkzeug needs to be true to run inside the docker
    # TODO: use better webserver
    app.run(port=NETAPP_PORT, host='0.0.0.0')


if __name__ == '__main__':
    main()
