import logging
import time

from celery import states as celery_states
from queue import Empty, Queue
from threading import Thread, Event

from era_5g_interface.interface_helpers import LatencyMeasurements


class ResultsReader(Thread):
    """
    Thread-based class which search the jobs queue for finished tasks and
    publish results to the robot.
    """

    def __init__(self, jobs_info_queue: Queue, send_function, **kw):
        """
        Constructor

        Args:
            name (str): Name of the thread
            jobs_info_queue (Queue): Queue with all to-be-processed jobs.
            app (_type_): A flask app for results publishing.
        """

        super().__init__(**kw)
        self.stop_event = Event()
        self._jobs_info_queue = jobs_info_queue
        self.send_function = send_function

        self.latency_measurements = LatencyMeasurements()

        self.pending_jobs = []

    def stop(self):
        self.stop_event.set()

    def run(self):
        """
        Periodically read the queue and search for finished jobs.
        """

        logging.info(f"Results reader thread for {self.name} is running.")

        while not self.stop_event.is_set():
            # Check for newly created jobs
            try:
                job = self._jobs_info_queue.get(timeout=0.01)
                self.pending_jobs.append(job)
            except Empty:
                pass

            # Check for completed jobs
            jobs_to_remove = set()
            for job in self.pending_jobs:

                if job.state == celery_states.SUCCESS:
                    jobs_to_remove.add(job)
                    (metadata, results) = job.get()
                    self.publish_results(metadata, results)
                    
                    # Remove results to free resources
                    # (note that this may or may not be desired, depending on application)
                    job.forget()  

                elif job.state == celery_states.REVOKED:
                    jobs_to_remove.add(job)

                elif job.state == celery_states.FAILURE:
                    jobs_to_remove.add(job)
                    logging.error(f"Task {job.task_id} failed.")
                    # Optional error handling

            # Remove completed jobs
            if len(jobs_to_remove):
                self.pending_jobs = [job for job in self.pending_jobs if job not in jobs_to_remove]
        
        self.cleanup()

    def cleanup(self):
        # Pull all remaining jobs from the queue
        while True:
            try:
                job = self._jobs_info_queue.get(timeout=0.1)
                self.pending_jobs.append(job)
            except Empty:
                break
        
        # Clean up jobs
        for job in self.pending_jobs:
            job.revoke()
            job.forget()


    def publish_results(self, metadata, detections):
        """
        Publishes the results to the robot

        Args:
            metadata (_type_): 5G-ERA Network Application specific metadata related to processed image.
            detections (_type_): The detection results.
        """

        send_timestamp = time.perf_counter_ns()
        self.latency_measurements.store_latency(send_timestamp - metadata["recv_timestamp"])

        r = {
            "timestamp": metadata["timestamp"],
             "recv_timestamp": metadata["recv_timestamp"],
             "send_timestamp": send_timestamp,
             "detections": detections,
        }
        self.send_function(data=r)

