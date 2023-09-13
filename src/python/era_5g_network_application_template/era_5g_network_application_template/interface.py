import binascii
import secrets
from queue import Queue
import cv2
import argparse
import os
import numpy as np
import flask_socketio
from flask import Flask, Response, request, session
from flask_session import Session
import logging

from era_5g_interface.task_handler_internal_q import TaskHandlerInternalQ
# the worker is needed when the network application should process the data in separated thread
#from era_5g_network_application_template.worker import Worker

# port of the netapp's server
NETAPP_PORT = os.getenv("NETAPP_PORT", 5896)

# flask initialization
app = Flask(__name__)
app.secret_key = secrets.token_hex()
app.config['SESSION_TYPE'] = 'filesystem'

# socketio initialization for sending results to the client
Session(app)
socketio = flask_socketio.SocketIO(app, manage_session=False, async_mode='threading')

# list of registered tasks (clients)
tasks = dict()

# queue with received data
# needed when the network application should process the data in separated thread
data_queue = Queue(30)

# the threaded worker to be used
worker_thread = None

class ArgFormatError(Exception):
    pass


@app.route('/register', methods=['POST'])
def register():
    """
    The endpoint, which is called to register new client (robot) with the network 
    application. It can receive the optional parameter using the "args" variable.

    """
    # get the network application configuration
    # args is dictionary with optional parameters
    args = request.get_json(silent=True)
    # register the robot's session
    session['registered'] = True

    # create an instance of task handler, which will be responsible for holding
    # the information of the robot and the connection to it
    task = TaskHandlerInternalQ(session.sid, data_queue, daemon=True)
    # we need to store the task into the dictionary
    tasks[session.sid] = task
    print(f"Client registered: {session.sid}")
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
        print(f"Client unregistered: {session_id}")

    return Response(status=204)


@app.route('/image', methods=['POST'])
def image_callback_http():
    """
    Allows to receive jpg-encoded image using the HTTP transport. 
    To reduce the HTTP overhead, the images are being sent in batches.
    """
    if 'registered' not in session:
        return Response('Need to call /register first.', 401)
    
    sid = session.sid
    task = tasks[sid]

    # unpack the timestamps from the timestamps array
    if "timestamps[]" in request.args:
        timestamps = request.args.to_dict(flat=False)['timestamps[]']
    else:
        timestamps = []
    index = 0
    # unpack the image files from the request
    for file in request.files.to_dict(flat=False)['files']:
        # convert string of image data to uint8
        nparr = np.frombuffer(file.read(), np.uint8)

        # store the image to the appropriate task
        # the image is not decoded here to make the callback as fast as possible
        #task.store_image({"sid": sid, 
        #                  "websocket_id": task.websocket_id, 
        #                  "timestamp": timestamps[index], 
        #                  "decoded": False}, 
        #                 nparr)
        index += 1
    return Response(status=204)

@app.route('/json', methods=['POST'])
def json_callback_http():
    if 'registered' not in session:
        return Response('Need to call /register first.', 401)
    
    sid = session.sid
    task = tasks[sid]
    data = request.get_json(silent=True)
    if not data:
        return Response("No data provided", status=400)
    
    # here the data could be processed or passed to the worker using the internal queue

    with app.app_context():
        # send results back to the client (robot)
        # flask_socketio.send(data: dict(), namespace='/results', to=task.websocket_id)
        # the format of the result is specific for the network application and it is not 
        # specified by the protocol
        flask_socketio.send({"result": data}, namespace='/results', to=task.websocket_id)
    return Response(status=200)

@socketio.on('image', namespace='/data')
def image_callback_websocket(data: dict):
    """
    Allows to receive jpg-encoded image using the websocket transport

    Args:
        data (dict): An image frame and (optionally) related timestamp in format:
            {'frame': 'bytes', 'timestamp': 'int'}

    Raises:
        ConnectionRefusedError: Raised when attempt for connection were made
            without registering first or frame was not passed in correct format.
    """
    logging.debug("A frame received using ws")
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
        # here the image frame could be processed or passed to the worker using the internal queue
        data["frame"]
        #task.store_image({"sid": session.sid, 
        #                  "websocket_id": task.websocket_id, 
        #                  "timestamp": timestamp, 
        #                  "decoded": False}, 
        #                 np.frombuffer(data["frame"], dtype=np.uint8))
    except (ValueError, binascii.Error) as error:
        logging.error(f"Failed to decode frame data: {error}")
        flask_socketio.emit("image_error", 
                            {"timestamp": timestamp, 
                             "error": f"Failed to decode frame data: {error}"}, 
                            namespace='/data', 
                            to=request.sid)

@socketio.on('json', namespace='/data')
def json_callback_websocket(data):
    """
    Allows to receive general json data using the websocket transport

    Args:
        data (dict): NetApp-specific json data

    Raises:
        ConnectionRefusedError: Raised when attempt for connection were made
            without registering first.
    """
    if 'registered' not in session:
        logging.error(f"Non-registered client tried to send data")
        flask_socketio.emit("json_error", 
                            {"error": "Need to call /register first."}, 
                            namespace='/data', 
                            to=request.sid)
        return
    sid = session.sid
    task = tasks[sid]

    # here the data could be processed or passed to the worker using the internal queue

    #data_queue.put(({"sid": sid, 
    #                 "websocket_id": task.websocket_id}, 
    #                data), 
    #               block=False)
    logging.debug(f"client with task id: {session.sid} sent data {data}")
    

@socketio.on('connect', namespace='/data')
def connect_data(auth):
    """Creates a websocket connection to the client for passing the data. 

    Raises:
        ConnectionRefusedError: Raised when attempt for connection were made
            without registering first.
    """

    if 'registered' not in session:
        raise ConnectionRefusedError('Need to call /register first.')
    
    print(f"Connected data. Session id: {session.sid}, ws_sid: {request.sid}")
    
    flask_socketio.send("you are connected", namespace='/data', to=request.sid)




@socketio.on('connect', namespace='/results')
def connect_results(auth):
    """
    Creates a websocket connection to the client for passing the results.

    Raises:
        ConnectionRefusedError: Raised when attempt for connection were made
            without registering first.
    """

    #print(f"connect {session.sid} {session}")
    #print(f"{request.sid} {request}")
    if 'registered' not in session:
        # TODO: disconnect?
        #flask_socketio.disconnect(request.sid, namespace="/results")
        raise ConnectionRefusedError('Need to call /register first.')

    sid = request.sid
    print(f"Client connected: session id: {session.sid}, websocket id: {sid}")
    tasks[session.sid].websocket_id = sid
    tasks[session.sid].start()
    flask_socketio.send("You are connected", namespace='/results', to=sid)


@socketio.on('disconnect', namespace='/results')
def disconnect_results():
    print(f"Client disconnected from /results namespace: session id: {session.sid}, websocket id: {request.sid}")

@socketio.on('disconnect', namespace='/data')
def disconnect_data():
    print(f"Client disconnected from /data namespace: session id: {session.sid}, websocket id: {request.sid}")



def main(args=None):
    
    logging.getLogger().setLevel(logging.DEBUG)

    worker_thread = Worker(data_queue, app)
    worker_thread.start()



    # runs the flask server
    # allow_unsafe_werkzeug needs to be true to run inside the docker
    # TODO: use better webserver
    socketio.run(app, port=NETAPP_PORT, host='0.0.0.0', allow_unsafe_werkzeug=True)


if __name__ == '__main__':
    main()
