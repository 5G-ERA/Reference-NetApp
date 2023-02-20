import os
import logging
import secrets
from queue import Queue
from era_5g_object_detection_distributed_interface.results_reader import ResultsReader

import flask_socketio
from era_5g_object_detection_distributed_interface.task_handler_gstreamer_rabbitmq_redis import \
    TaskHandlerGstreamerRabbitmqRedis as TaskHandler
from era_5g_netapp_interface.common import get_logger
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

# list of available ports for gstreamer communication - they needs to be exposed when running in docker / k8s
free_ports = [5001, 5002, 5003]

# list of registered tasks
tasks = dict()

# queue with all celery jobs for results retrieval
jobs_info_queue = Queue(1024)

logger = get_logger(log_level=logging.INFO)

@app.route('/register', methods=['GET'])
def register():
    """
    Needs to be called before an attempt to open WS is made.

    Returns:
        _type_: The port used for gstreamer communication.
    """
    if not free_ports:
        return {"error": "Not enough resources"}, 503

    # gets a free gstreamer port (if there is any)
    port = free_ports.pop(0)
    session['registered'] = True
    # creates task handler and save it under the session_id for future reference
    task = TaskHandler(logger, session.sid, port, jobs_info_queue)     
    tasks[session.sid] = task

    print(f"Client registered: {session.sid}")
    return {"port": port}, 200


@app.route('/unregister', methods=['GET'])
def unregister():
    """_
    Disconnects the websocket and removes the task from the memory.

    Returns:
        _type_: 204 status
    """
    session_id = session.sid

    if session.pop('registered', None):
        task = tasks.pop(session.sid)
        flask_socketio.disconnect(task.websocket_id, namespace="/results")
        free_ports.append(task.port)
        task.stop()
        print(f"Client unregistered: {session_id}")

        
    return Response(status=204)


@socketio.on('connect', namespace='/results')
def connect(auth):
    """
    Creates a websocket connection to the client for passing the results.


    Raises:
        ConnectionRefusedError: Raised when attempt for connection were made
            without registering first.
    """
    if 'registered' not in session:
        raise ConnectionRefusedError('Need to call /register first.')

    sid = request.sid

    print(f"Connected. Session id: {session.sid}, ws_sid: {sid}")
    # saves the websocket_id for sending the results
    tasks[session.sid].websocket_id = sid
    tasks[session.sid].start(daemon=True)
    flask_socketio.send("you are connected", namespace='/results', to=sid)


@socketio.event
def disconnect(sid):
    print('disconnect ', sid)


def main(args=None):
    # creates a results reader, which periodicaly reads status of jobs in jobs_info_queue
    # to find finished jobs and pass the results to the robot
    results_reader = ResultsReader(logger, "results_reader", jobs_info_queue, app)
    results_reader.start(daemon=True)
    
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
