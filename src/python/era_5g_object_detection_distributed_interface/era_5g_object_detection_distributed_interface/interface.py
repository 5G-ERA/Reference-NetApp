import base64
import binascii
import os
import logging
import secrets
import time
from queue import Queue

import numpy as np
from era_5g_object_detection_distributed_interface.results_reader import ResultsReader

import flask_socketio

from era_5g_object_detection_distributed_interface.task_handler_distributed import \
    TaskHandlerDistributed
from flask import Flask, Response, request, session

from flask_session import Session

# port of the netapp's server
NETAPP_PORT = os.getenv("NETAPP_PORT", 5896)

# flask initialization
app = Flask(__name__)
app.secret_key = secrets.token_hex()
app.config['SESSION_TYPE'] = 'filesystem'

# socketio initialization for sending results to the client
Session(app)
socketio = flask_socketio.SocketIO(app, manage_session=False, async_mode='threading')

# list of available ports for gstreamer communication - they need to be exposed when running in docker / k8s
free_ports = [5001, 5002, 5003]

# list of registered tasks
tasks = dict()

# queue with all celery jobs for results retrieval
jobs_info_queue = Queue(1024)


@app.route('/register', methods=['POST'])
def register():
    """
    Needs to be called before an attempt to open WS is made.

    Returns:
        _type_: The port used for gstreamer communication.
    """

    args = request.get_json(silent=True)
    gstreamer = False
    if args:
        gstreamer = args.get("gstreamer", False)

    if gstreamer and not free_ports:
        return {"error": "Not enough resources"}, 503
    if not free_ports:
        return {"error": "Not enough resources"}, 503
    
    session['registered'] = True

    # select the appropriate task handler, depends on whether the client wants to use
    # gstreamer to pass the images or not 
    if gstreamer:
        port = free_ports.pop(0)
        task = TaskHandlerGstreamerDistributed(session.sid, port, jobs_info_queue, daemon=True)
    else:
        task = TaskHandlerDistributed(session.sid, jobs_info_queue, daemon=True)

    tasks[session.sid] = task
    print(f"Client registered: {session.sid}")
    if gstreamer:
        return {"port": port}, 200
    else:
        return Response(status=204)

@app.route('/unregister', methods=['POST'])
def unregister():
    """_
    Disconnects the websocket and removes the task from the memory.

    Returns:
        _type_: 204 status
    """
    session_id = session.sid

    if session.pop('registered', None):
        task = tasks.pop(session.sid)
        task.stop()
        flask_socketio.disconnect(task.websocket_id, namespace="/results")
        if isinstance(task, TaskHandlerGstreamer):
            free_ports.append(task.port)
        print(f"Client unregistered: {session_id}")

    return Response(status=204)

@app.route('/image', methods=['POST'])
def image_callback_http():
    """
    Allows to receive jpg-encoded image using the HTTP transport
    """

    recv_timestamp = time.time_ns()

    if 'registered' not in session:
        return Response('Need to call /register first.', 401)
    
    sid = session.sid
    task = tasks[sid]

    if "timestamps[]" in request.args:
        timestamps = request.args.to_dict(flat=False)['timestamps[]']
    else:
        timestamps = []
    # convert string of image data to uint8
    index = 0
    for file in request.files.to_dict(flat=False)['files']:
        # print(file)
        nparr = np.frombuffer(file.read(), np.uint8)

        # store the image to the appropriate task
        # the image is not decoded here to make the callback as fast as possible
        task.store_image({"sid": sid, 
                          "websocket_id": task.websocket_id, 
                          "timestamp": timestamps[index],
                          "recv_timestamp": recv_timestamp,
                          "decoded": False}, 
                         nparr)
        index += 1
    return Response(status=204)


@socketio.on('image', namespace='/data')
def image_callback_websocket(data: dict):
    """
    Allows to receive jpg-encoded image using the websocket transport

    Args:
        data (dict): A base64 encoded image frame and (optionally) related timestamp in format:
            {'frame': 'base64data', 'timestamp': 'int'}

    Raises:
        ConnectionRefusedError: Raised when attempt for connection were made
            without registering first or frame was not passed in correct format.
    """
    logging.debug("A frame recieved using ws")
    recv_timestamp = time.time_ns()

    if 'timestamp' in data:
        timestamp = data['timestamp']
    else:
        logging.debug("Timestamp not set, setting default value")
        timestamp = 0
    if 'registered' not in session:
        logging.error(f"Non-registered client tried to send data")
        flask_socketio.emit("image_error", 
                            {"timestamp": timestamp, 
                             "error": "Need to call /register first."}, 
                            namespace='/data', 
                            to=request.sid)
        return
    if 'frame' not in data:
        logging.error(f"Data does not contain frame.")
        flask_socketio.emit("image_error", 
                            {"timestamp": timestamp, 
                             "error": "Data does not contain frame."}, 
                            namespace='/data', 
                            to=request.sid)
        return

    task = tasks[session.sid]
    try:
        frame = base64.b64decode(data["frame"])
        task.store_image({"sid": session.sid, 
                          "websocket_id": task.websocket_id, 
                          "timestamp": timestamp,
                          "recv_timestamp": recv_timestamp,
                          "decoded": False}, 
                         np.frombuffer(frame, dtype=np.uint8))
    except (ValueError, binascii.Error) as error:
        logging.error(f"Failed to decode frame data: {error}")
        flask_socketio.emit("image_error", 
                            {"timestamp": timestamp, 
                             "error": f"Failed to decode frame data: {error}"}, 
                            namespace='/data', 
                            to=request.sid)

@socketio.on('connect', namespace='/results')
def connect_results(auth):
    """
    Creates a websocket connection to the client for passing the results.

    Raises:
        ConnectionRefusedError: Raised when attempt for connection were made
            without registering first.
    """

    if 'registered' not in session:
        raise ConnectionRefusedError('Need to call /register first.')

    sid = request.sid
    print(f"Client connected: session id: {session.sid}, websocket id: {sid}")
    # saves the websocket_id for sending the results
    tasks[session.sid].websocket_id = sid
    tasks[session.sid].start()
    flask_socketio.send("you are connected", namespace='/results', to=sid)

@socketio.on('connect', namespace='/data')
def connect_data(auth):
    """_summary_
    Creates a websocket connection to the client for passing the data.

    Raises:
        ConnectionRefusedError: Raised when attempt for connection were made
            without registering first.
    """
    print("connectes")
    if 'registered' not in session:
        raise ConnectionRefusedError('Need to call /register first.')
    
    print(f"Connected data. Session id: {session.sid}, ws_sid: {request.sid}")
    
    flask_socketio.send("you are connected", namespace='/data', to=request.sid)


@socketio.event
def disconnect_results(sid):
    print(f"Client disconnected from /results namespace: session id: {session.sid}, websocket id: {request.sid}")

@socketio.on('disconnect', namespace='/data')
def disconnect_data():
    print(f"Client disconnected from /data namespace: session id: {session.sid}, websocket id: {request.sid}")

def main(args=None):
    # creates a results' reader, which periodically reads status of jobs in jobs_info_queue
    # to find finished jobs and pass the results to the robot

    logging.getLogger().setLevel(logging.INFO)

    results_reader = ResultsReader(jobs_info_queue, app, name="results_reader", daemon=True)
    results_reader.start()

    # Estimate new queue size based on maximum latency
    """avg_latency = latency_measurements.get_avg_latency() # obtained from detector init warmup
    if avg_latency != 0:  # warmup can be skipped
        new_queue_size = int(max_latency / avg_latency)
        image_queue = Queue(new_queue_size)
        detector_thread.input_queue = image_queue"""

    # runs the flask server
    # allow_unsafe_werkzeug needs to be true to run inside the docker
    # TODO: use better webserver
    socketio.run(app, port=NETAPP_PORT, host='0.0.0.0', allow_unsafe_werkzeug=True)


if __name__ == '__main__':
    main()
