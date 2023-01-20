#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from time import sleep

from queue import Empty, Full, Queue
import json
from threading import Thread, Event
import logging
import ros_numpy

class ResultsReader(Thread):

    def __init__(self, logger, name, queue, result_publisher):
        super(ResultsReader, self).__init__()

        self.stopped = False
        self.time = None
        self.fps = 0.0
        self.logger = logger
        self._queue = queue
        self._result_publisher = result_publisher
        self.logger.info("Results reader thread is created.")
        # Determine if results contain masks
        # model_variant = os.getenv("NETAPP_MODEL_VARIANT")
        #self.with_masks = MODEL_VARIANTS[model_variant]['with_masks']
        

        

    

    def publish_results(self, data, with_mask=False):
        metadata, raw_results = data

        results = dict()
        results["header"] = dict()
        stamp = metadata["header"].stamp
        results["header"]["stamp"] = "%d.%d" % (stamp.sec, stamp.nanosec)
        results["header"]["frame_id"] = metadata["header"].frame_id

        results["detections"] = []

        for raw_result in raw_results:
            det = dict()
            if with_mask:
                bbox, score, cls_id, cls_name, mask = raw_result
                det["mask"] = mask
            else:
                bbox, score, cls_id, cls_name = raw_result

            det["bbox"] = [float(i) for i in bbox]
            det["score"] = float(score)
            det["class"] = int(cls_id)
            det["class_name"] = str(cls_name)

            results["detections"].append(det)

        msg = String()
        msg.data = json.dumps(results)
        self._result_publisher.publish(msg)

class ImageConnector:
    def __init__(self):
        self.img_sub = rospy.Subscriber("~images", Image, self.callback_img)

        self._queue = Queue()
        # Create results publisher
        self._result_publisher = rospy.Publisher("~results", String, queue_size=10)
        logger = logging.getLogger()
        logger.setLevel(logging.INFO)
        

        self.with_masks = False
        self.jobs_in_process = []


    
    def callback_img(self, msg):
        current_frame = ros_numpy.numpify(msg)
        # Create metadata for image
        metadata = {"node_name": "image_connector", "header": {"seq": int(msg.header.seq), "stamp": {"sec": int(msg.header.stamp.secs), "nsec": int(msg.header.stamp.nsecs)}, "frame_id": msg.header.frame_id}}

        job_data = (metadata, current_frame)

        # Start asynchronous Celery task
        job = detector_task.delay(job_data)
        # Pass info about the task to the thread for reading results
        try:
            self._queue.put(job, block=False)
        except Full:
            rospy.logwarn("Internal queue full, skipping data")
            job.revoke()
            return


    def run(self):
        while not rospy.is_shutdown():
            # Check for newly created jobs
            try:
                job = self._queue.get_nowait()
                self.jobs_in_process.append(job)
            except Empty:
                pass

            # Check for completed jobs
            jobs_to_remove = set()
            for job in self.jobs_in_process:
                if job.state == "SUCCESS":
                    jobs_to_remove.add(job)
                    result = job.get()
                    self.publish_results(result, self.with_masks)

                elif job.state == "REVOKED":
                    jobs_to_remove.add(job)

                elif job.state == "FAILURE":
                    jobs_to_remove.add(job)
                    

            # Remove completed jobs
            if len(jobs_to_remove):
                self.jobs_in_process = [job for job in self.jobs_in_process if job not in jobs_to_remove]
            else:
                sleep(0.02)
        

    def publish_results(self, data, with_mask=False):
        metadata, raw_results = data

        results = dict()
        results["header"] = dict()
        stamp = metadata["header"]["stamp"]
        results["header"]["stamp"] = "%d.%d" % (stamp["sec"], stamp["nsec"])
        results["header"]["frame_id"] = metadata["header"]["frame_id"]

        results["detections"] = []

        for raw_result in raw_results:
            det = dict()
            if with_mask:
                bbox, score, cls_id, cls_name, mask = raw_result
                det["mask"] = mask
            else:
                bbox, score, cls_id, cls_name = raw_result

            det["bbox"] = [float(i) for i in bbox]
            det["score"] = float(score)
            det["class"] = int(cls_id)
            det["class_name"] = str(cls_name)

            results["detections"].append(det)

        msg = String()
        msg.data = json.dumps(results)
        self._result_publisher.publish(msg)

if __name__ == '__main__':
    try:
        rospy.init_node("image_connector")
        from era_5g_object_detection_distributed.ml_service_worker import detector_task
        node = ImageConnector()
        node.run()
    except rospy.ROSInterruptException:
        pass
