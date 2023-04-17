
import base64  # for decoding masks
import cv2
import logging
import time

from queue import Queue, Empty
from threading import Event, Thread

import numpy as np
import pycocotools.mask as masks_util  # for decoding masks


DEBUG_PRINT_SCORE = False  # useful for FPS detector
DEBUG_PRINT_DELAY = False  # prints the delay between capturing image and receiving the results
DEBUG_DRAW_MASKS = True  # draw segmentation masks (if provided by detector)


class ResultsViewer(Thread):
    def __init__(self, image_storage, results_queue, **kw) -> None:
        super().__init__(**kw)
        self.image_storage = image_storage
        self.results_queue = results_queue
        self.stop_event = Event()
        self.index = 0

    def stop(self) -> None:
        self.stop_event.set()

    def run(self) -> None:
        logging.info("Thread %s: starting", self.name)
        while not self.stop_event.is_set():
            try:
                results = self.results_queue.get(timeout=1)
            except Empty:
                continue
            timestamp_str = results["timestamp"]
            timestamp = int(timestamp_str)
            if DEBUG_PRINT_DELAY:
                time_now = time.time_ns()
                print(f"{(time_now - timestamp) * 1.0e-9:.3f}s delay")
            try:
                frame = self.image_storage.pop(timestamp_str)
                detections = results["detections"]
                for d in detections:
                    score = float(d["score"])
                    if DEBUG_PRINT_SCORE and score > 0:
                        print(score)
                    cls_name = d["class_name"]
                    # Draw detection into frame.
                    x1, y1, x2, y2 = [int(coord) for coord in d["bbox"]]
                    if cls_name == "person":
                        color = (255, 1, 252)
                    elif cls_name in ["car", "truck"]:
                        color = (255, 255, 0)
                    else:
                        color = (0, 255, 0)
                    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    cv2.putText(
                        frame,
                        f"{cls_name} ({score * 100:.0f})%",
                        (x1, y1 - 5),
                        font,
                        1,
                        color,
                        1,
                        cv2.LINE_AA,
                    )

                    if "mask" in d and DEBUG_DRAW_MASKS:
                        encoded_mask = d["mask"]
                        encoded_mask["counts"] = base64.b64decode(encoded_mask["counts"])  # Base64 decode
                        mask = masks_util.decode(encoded_mask).astype(bool)  # RLE decode
                        color_mask = np.random.randint(0, 256, (1, 3), dtype=np.uint8)
                        frame[mask] = frame[mask] * 0.5 + color_mask * 0.5

                try:
                    cv2.imshow("Results", frame)
                    cv2.waitKey(1)
                except Exception as ex:
                    print(ex)
                self.results_queue.task_done()
            except KeyError as ex:
                print(ex)
