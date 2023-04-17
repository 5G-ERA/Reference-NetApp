from queue import Empty, Queue
import flask_socketio
import logging
from threading import Event, Thread


class Worker(Thread):
    """
    Worker object for data processing in standalone variant. Reads 
    data from passed queue, performs detection and returns results using
    the flask app. 
    """

    def __init__(self, data_queue: Queue, app, **kw):
        """
        Constructor

        Args:
            data_queue (Queue): The queue with all to-be-processed data
            app (_type_): The flask app for results publishing
        """

        super().__init__(**kw)
        self.data_queue = data_queue
        self.stop_event = Event()
        self.app = app


    def stop(self):
        self.stop_event.set()
        
    def run(self):
        """
        Periodically reads data from python internal queue process them.
        """

        logging.debug(f"{self.name} thread is running.")

        while not self.stop_event.is_set():
            # Get data and metadata from input queue
            try:
                metadata, data = self.data_queue.get(block=True)
            except Empty:
                continue
            # process the data
            result = self.process_data(data)
            # publish the data
            self.publish_results(result, metadata)

    def publish_results(self, results, metadata):
        """
        Publishes the results to the robot

        Args:
            metadata (_type_): NetApp-specific metadata related to processed image.
            results (_type_): The results of the detection.
        """

        #process the raw results
        # results = prepare_results_for_publication(results)

        # use the flask app to return the results
        with self.app.app_context():
            # print(f"publish_results to: {metadata['websocket_id']} flask_socketio.send: {r}")
            flask_socketio.send(results, namespace='/results', to=metadata["websocket_id"])

    def process_data(self, data):
        return data