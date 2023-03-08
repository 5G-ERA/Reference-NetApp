from __future__ import annotations

import base64  # for decoding masks
import logging
import math
import os
import signal
import time
import traceback
from queue import Queue
from threading import Event, Thread
from types import FrameType
from typing import Any, Dict, Optional

import cv2
import numpy as np
import pycocotools.mask as masks_util  # for decoding masks

from era_5g_client.client import NetAppClient
from era_5g_client.exceptions import FailedToConnect

image_storage: Dict[str, np.ndarray] = dict()
results_storage: Queue[Dict[str, Any]] = Queue()
stopped = False

DEBUG_PRINT_SCORE = False  # useful for FPS detector
DEBUG_PRINT_DELAY = False  # prints the delay between capturing image and receiving the results
DEBUG_DRAW_MASKS = True  # draw segmentation masks (if provided by detector)

# Video from source flag
FROM_SOURCE = False
# ip address or hostname of the computer, where the netapp is deployed
NETAPP_ADDRESS = os.getenv("NETAPP_ADDRESS", "127.0.0.1")
# port of the netapp's server
NETAPP_PORT = int(os.getenv("NETAPP_PORT", 5896))
# test video file
try:
    TEST_VIDEO_FILE = os.environ["TEST_VIDEO_FILE"]
except KeyError as e:
    raise Exception(f"Failed to run example, env variable {e} not set.")

if not os.path.isfile(TEST_VIDEO_FILE):
    raise Exception("TEST_VIDEO_FILE does not contain valid path to a file.")


class ResultsViewer(Thread):
    def __init__(self, **kw) -> None:
        super().__init__(**kw)
        self.stop_event = Event()

    def stop(self) -> None:
        self.stop_event.set()

    def run(self) -> None:
        logging.info("Thread %s: starting", self.name)
        while not self.stop_event.is_set():
            if not results_storage.empty():
                results = results_storage.get(timeout=1)
                timestamp_str = results["timestamp"]
                timestamp = int(timestamp_str)
                if DEBUG_PRINT_DELAY:
                    time_now = math.floor(time.time() * 100)
                    print(f"{(time_now - timestamp) / 100.}s delay")
                try:
                    frame = image_storage.pop(timestamp_str)
                    detections = results["detections"]
                    for d in detections:
                        score = float(d["score"])
                        if DEBUG_PRINT_SCORE and score > 0:
                            print(score)
                        cls_name = d["class_name"]
                        # Draw detection into frame.
                        x1, y1, x2, y2 = [int(coord) for coord in d["bbox"]]
                        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 1)
                        font = cv2.FONT_HERSHEY_SIMPLEX
                        cv2.putText(
                            frame,
                            f"{cls_name} ({score * 100:.0f})%",
                            (x1, y1 - 5),
                            font,
                            0.5,
                            (0, 255, 0),
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
                    results_storage.task_done()
                except KeyError as ex:
                    print(ex)


def get_results(results: Dict[str, Any]) -> None:
    """Callback which process the results from the NetApp.

    Args:
        results (str): The results in json format
    """

    print(results)
    if "timestamp" in results:
        results_storage.put(results, block=False)
    pass


def main() -> None:
    """Creates the client class and starts the data transfer."""

    results_viewer = ResultsViewer(name="test_client_http_viewer", daemon=True)
    results_viewer.start()

    logging.getLogger().setLevel(logging.INFO)

    client = None
    global stopped
    stopped = False

    def signal_handler(sig: int, frame: Optional[FrameType]) -> None:
        global stopped
        stopped = True
        results_viewer.stop()
        print(f"Terminating ({signal.Signals(sig).name})...")

    signal.signal(signal.SIGTERM, signal_handler)
    signal.signal(signal.SIGINT, signal_handler)

    try:
        # creates the NetApp client with gstreamer extension
        client = NetAppClient("", "", "", "", True, get_results, False, False, NETAPP_ADDRESS, NETAPP_PORT)
        # register the client with the NetApp
        client.register()
        if FROM_SOURCE:
            # creates a video capture to pass images to the NetApp either from webcam ...
            cap = cv2.VideoCapture(0)
            if not cap.isOpened():
                raise Exception("Cannot open camera")
        else:
            # or from video file
            cap = cv2.VideoCapture(TEST_VIDEO_FILE)
            if not cap.isOpened():
                raise Exception("Cannot open video file")

        while not stopped:
            ret, frame = cap.read()
            timestamp = math.floor(time.time() * 100)
            if not ret:
                break
            resized = cv2.resize(frame, (640, 480), interpolation=cv2.INTER_AREA)
            timestamp_str = str(timestamp)
            image_storage[timestamp_str] = resized
            client.send_image_http(resized, timestamp_str, 5)

    except FailedToConnect as ex:
        print(f"Failed to connect to server ({ex})")
    except KeyboardInterrupt:
        print("Terminating...")
    except Exception as ex:
        traceback.print_exc()
        print(f"Failed to create client instance ({ex})")
    finally:
        results_viewer.stop()
        if client is not None:
            client.disconnect()


if __name__ == "__main__":
    main()
