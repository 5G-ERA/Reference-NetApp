
import argparse
import cv2
import logging
import os
import time
import numpy as np

from queue import Queue, Full

from era_5g_object_detection_standalone.worker_mmdet import MMDetectorWorker 
from utils.rate_timer import RateTimer
from client_http_no_middleware import ResultsViewer


class WrappedDetectorWorker(MMDetectorWorker):
    """Wrapper that publishes results to a queue instead of interface.
    
    The publish_results method of the original worker 
    is modified to put the detection results into local
    queue instead of sending them over network interface.
    """

    def __init__(self, output_queue, verbose_processing, **kw_args) -> None:
        super().__init__(**kw_args)
        self.output_queue = output_queue
        self.verbose_processing = verbose_processing

        # warmup 
        empty_frame = np.zeros((480,640,3))
        for _ in range(3):
            self.process_image(empty_frame)

    def publish_results(self, results, metadata) -> None:
        """Overrides the original method - send data to output queue."""

        detections = list()
        if results is not None:
            for (bbox, score, cls_id, cls_name) in results:
                det = dict()
                det["bbox"] = [float(i) for i in bbox]
                det["score"] = float(score)
                det["class"] = int(cls_id)
                det["class_name"] = str(cls_name)

                detections.append(det)

            r = {"timestamp": metadata["timestamp"],
                 "detections": detections}
            
            if self.output_queue is not None:
                try:
                    self.output_queue.put(r, block=False)
                except Full:
                    logging.warning("Output queue full!")
            if self.verbose_processing:
                logging.info(f"Frame with timestamp {metadata['timestamp']} processed.")
                

def main():
    parser = argparse.ArgumentParser(description='Example with direct local usage of mmcv detector.')
    parser.add_argument("-s", "--show_results",
        default=False, action="store_true",
        help="Show window with visualization of detection results. Defaults to False."
        )
    parser.add_argument("-v", "--verbose",
        default=False, action="store_true",
        help="Print information about processed data. Defaults to False."
        )
    args = parser.parse_args()

    logging.getLogger().setLevel(logging.DEBUG)

    image_storage = dict()
    image_queue = Queue(50)
    output_queue = None

    # create results viewer together with output_queue
    if args.show_results:
        output_queue = Queue(50)
        results_viewer = ResultsViewer(image_storage, output_queue, name="test_viewer", daemon=True)
        results_viewer.start()

    # init detector
    detector_thread = WrappedDetectorWorker(output_queue, args.verbose, image_queue=image_queue, app=None, name="Detector", daemon=True)
    detector_thread.start()

    # open video file
    try:
        TEST_VIDEO_FILE = os.environ["TEST_VIDEO_FILE"]
    except KeyError as e:
        raise Exception(f"Failed to run example, env variable {e} not set.")
    
    cap = cv2.VideoCapture(TEST_VIDEO_FILE)
    if not cap.isOpened():
        raise Exception("Cannot open video file")

    # create timer to ensure required fps speed of the sending loop
    fps = cap.get(cv2.CAP_PROP_FPS)
    logging.info(f"Using RateTimer with {fps} FPS.")
    rate_timer = RateTimer(rate=fps, iteration_miss_warning=True)

    while True:
        ret, frame = cap.read()
        timestamp = time.time_ns()
        if not ret:
            break
        resized_img = cv2.resize(frame, (640, 480), interpolation=cv2.INTER_AREA)
        timestamp_str = str(timestamp)
        image_storage[timestamp_str] = resized_img
        metadata = {"timestamp": timestamp_str}

        rate_timer.sleep()  # sleep until next frame should be sent (with given fps)

        try:
            image_queue.put((metadata, resized_img), block=False)
        except Full:
            logging.warning("Input queue full!")


if __name__ == "__main__":
    main()
