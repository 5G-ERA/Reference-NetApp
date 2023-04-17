from __future__ import annotations

import argparse
import csv
import logging
import math
import os
import signal
import time
import traceback

from datetime import datetime
from queue import Queue, Empty
from threading import Event, Thread
from types import FrameType
from typing import Any, Dict, Optional

import cv2
import numpy as np

from era_5g_client.client_base import NetAppClientBase
from era_5g_client.exceptions import FailedToConnect

from era_5g_client.dataclasses import NetAppLocation

from utils.rate_timer import RateTimer
from utils.results_viewer import ResultsViewer

image_storage: Dict[str, np.ndarray] = dict()
results_storage: Queue[Dict[str, Any]] = Queue()
time_measurements = []
stopped = False
verbose = False


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


def get_results(results: Dict[str, Any]) -> None:
    """Callback which process the results from the NetApp.

    Args:
        results (str): The results in json format
    """

    if verbose:
        print(results)

    if "timestamp" not in results:
        return

    if results_storage is not None:
        results_storage.put(results, block=False)
    

def main() -> None:
    """Creates the client class and starts the data transfer."""

    parser = argparse.ArgumentParser(description='Example client communication without middleware.')
    parser.add_argument("-n", "--no-results",
        default=False, action="store_true",
        help="Do not show window with visualization of detection results. Defaults to False."
        )
    parser.add_argument("-v", "--verbose",
        default=False, action="store_true",
        help="Print information about processed data. Defaults to False."
        )
    args = parser.parse_args()
    global verbose
    verbose = args.verbose

    logging.getLogger().setLevel(logging.INFO)

    global results_storage
    if args.no_results:
        results_storage = None
    else:
        results_viewer = ResultsViewer(image_storage, results_storage, name="test_client_http_viewer", daemon=True)
        results_viewer.start()

    client = None
    global stopped
    stopped = False

    def signal_handler(sig: int, frame: Optional[FrameType]) -> None:
        global stopped
        stopped = True
        if not args.no_results:
            results_viewer.stop()
        print(f"Terminating ({signal.Signals(sig).name})...")

    signal.signal(signal.SIGTERM, signal_handler)
    signal.signal(signal.SIGINT, signal_handler)

    try:
        # creates an instance of NetApp client with results callback
        client = NetAppClientBase(get_results)
        # register with an ad-hoc deployed NetApp
        client.register(NetAppLocation(NETAPP_ADDRESS, NETAPP_PORT))
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

        # create timer to ensure required fps speed of the sending loop
        fps = cap.get(cv2.CAP_PROP_FPS)
        logging.info(f"Using RateTimer with {fps} FPS.")
        rate_timer = RateTimer(rate=fps, iteration_miss_warning=True)

        while not stopped:
            ret, frame = cap.read()
            timestamp = time.time_ns()
            if not ret:
                break
            resized = cv2.resize(frame, (640, 480), interpolation=cv2.INTER_AREA)
            timestamp_str = str(timestamp)
            if not args.no_results:
                image_storage[timestamp_str] = resized

            rate_timer.sleep()  # sleep until next frame should be sent (with given fps)
            client.send_image_http(resized, timestamp_str, 5)

    except FailedToConnect as ex:
        print(f"Failed to connect to server ({ex})")
    except KeyboardInterrupt:
        print("Terminating...")
    except Exception as ex:
        traceback.print_exc()
        print(f"Failed to create client instance ({ex})")
    finally:
        if client is not None:
            client.disconnect()


if __name__ == "__main__":
    main()
