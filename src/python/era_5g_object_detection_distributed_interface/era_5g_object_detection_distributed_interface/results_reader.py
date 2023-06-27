import json
from queue import Empty, Queue
import string
from time import sleep
from threading import Thread, Event
import flask_socketio
import logging
import time

from era_5g_interface.interface_helpers import LatencyMeasurements


class ResultsReader(Thread):
    """
    Thread-based class which search the jobs queue for finished tasks and
    publish results to the robot.
    """

    def __init__(self, jobs_info_queue: Queue, app, **kw):
        """
        Constructor

        Args:
            name (str): Name of the thread
                jobs_info_queue (Queue): Queue with all to-be-processed jobs.
            app (_type_): A flask app for results publishing.
        """

        super().__init__(**kw)
        self.stop_event = Event()
        self.time = None
        self.fps = 0.0
        self._jobs_info_queue = jobs_info_queue
        self.app = app

        self.latency_measurements = LatencyMeasurements()

        self.jobs_in_process = []

    def stop(self):
        self.stop_event.set()

    def run(self):
        """
        Periodically reads the queue and search for finished jobs.
        """

        logging.info(f"Results reader thread for {self.name} is running.")

        while not self.stop_event.is_set():
            # Check for newly created jobs
            try:
                job = self._jobs_info_queue.get_nowait()
                self.jobs_in_process.append(job)
            except Empty:
                pass

            # Check for completed jobs
            jobs_to_remove = set()
            for job in self.jobs_in_process:

                if job.state == "SUCCESS":
                    jobs_to_remove.add(job)
                    (metadata, results) = job.get()
                    self.publish_results(metadata, results)

                elif job.state == "REVOKED":
                    jobs_to_remove.add(job)

                elif job.state == "FAILURE":
                    jobs_to_remove.add(job)
                    logging.error(f"Task {job.task_id} failed.")
                    # TODO: optional error handling

            # Remove completed jobs
            if len(jobs_to_remove):
                self.jobs_in_process = [job for job in self.jobs_in_process if job not in jobs_to_remove]
            else:
                sleep(0.01)

        print(f"job called: {job}")

    def publish_results(self, metadata, results):
        """
        Publishes the results to the robot

        Args:
            metadata (_type_): NetApp-specific metadata related to processed image.
            results (_type_): The results of the detection.
        """

        detections = list()
        if results is not None:
            for (bbox, score, cls_id, cls_name) in results:
                det = dict()
                det["bbox"] = [float(i) for i in bbox]
                det["score"] = float(score)
                det["class"] = int(cls_id)
                det["class_name"] = str(cls_name)

                detections.append(det)
            
            send_timestamp = time.time_ns()

            self.latency_measurements.store_latency(send_timestamp - metadata["recv_timestamp"])

            r = {"timestamp": metadata["timestamp"],
                 "recv_timestamp": metadata["recv_timestamp"],
                 "send_timestamp": send_timestamp,
                 "detections": detections}
        

            # use the flask app to return the results
            with self.app.app_context():
                flask_socketio.send(r, namespace='/results', to=metadata["websocket_id"])
