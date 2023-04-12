
import argparse
import csv
import cv2
import logging
import os
import time
import numpy as np

from datetime import datetime
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

    def __init__(self, output_queue, time_measurements, verbose_processing, **kw_args) -> None:
        super().__init__(**kw_args)
        self.output_queue = output_queue
        self.time_measurements = time_measurements
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

            # store time measurements
            send_timestamp = time.time_ns()
            self.time_measurements.append([
                metadata["timestamp"],
                metadata["timestamp"],  # placeholder instead of recv_timestamp
                send_timestamp,  
                send_timestamp  # placeholder instead of final_timestamp
                ])

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
    parser.add_argument("-n", "--no-results",
        default=False, action="store_true",
        help="Do not show window with visualization of detection results. Defaults to False."
        )
    parser.add_argument("-v", "--verbose",
        default=False, action="store_true",
        help="Print information about processed data. Defaults to False."
        )
    parser.add_argument("-o", "--out-csv-dir",
        default=None, 
        help="Path to a directory where output csv file with results of time measurements will be stored. "
            "If not set, no measurement results are saved."
        )
    parser.add_argument("-p", "--out-prefix",
        default="direct_detector_test_",
        help="Prefix of output csv file with measurements. The file is suffixed with current time."
        )
    args = parser.parse_args()

    logging.getLogger().setLevel(logging.DEBUG)
    current_date_time = datetime.now().strftime("%Y-%d-%m_%H-%M-%S")

    image_storage = dict()
    image_queue = Queue(50)
    output_queue = None
    time_measurements = []

    # make sure that output dir exists 
    if args.out_csv_dir is not None:
        os.makedirs(args.out_csv_dir, exist_ok=True)
        # make sure the dir is ok for writing
        assert os.access(args.out_csv_dir, os.W_OK | os.X_OK) 

    # create results viewer together with output_queue
    if not args.no_results:
        output_queue = Queue(50)
        results_viewer = ResultsViewer(image_storage, output_queue, name="test_viewer", daemon=True)
        results_viewer.start()

    # init detector
    detector_thread = WrappedDetectorWorker(output_queue, time_measurements,
                                            args.verbose, image_queue=image_queue, 
                                            app=None, name="Detector", daemon=True)
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
        if not args.no_results:
            image_storage[timestamp_str] = resized_img
        metadata = {"timestamp": timestamp_str}

        rate_timer.sleep()  # sleep until next frame should be sent (with given fps)

        try:
            image_queue.put((metadata, resized_img), block=False)
        except Full:
            logging.warning("Input queue full!")

    # save measured times to csv file
    if args.out_csv_dir is not None:
        out_csv_filename = f"{args.out_prefix}{current_date_time}"
        out_csv_filepath = os.path.join(args.out_csv_dir, out_csv_filename)
        with open(out_csv_filepath, "w", newline='') as csv_file:
            csv_writer = csv.writer(csv_file)
            csv_writer.writerows(time_measurements)


if __name__ == "__main__":
    main()
