import traceback
import math
import time
from threading import Thread, Event
import cv2
import signal
from queue import Queue
import logging
import os

from era_5g_netapp_client.client import FailedToConnect, NetAppClient

image_storage = dict()
results_storage = Queue()

DEBUG_PRINT_SCORE = False  # useful for FPS detector
DEBUG_PRINT_DELAY = False  # prints the delay between capturing image and receiving the results

# Video from source flag
FROM_SOURCE = False
# ip address or hostname of the computer, where the netapp is deployed
NETAPP_ADDRESS = os.getenv("NETAPP_ADDRESS", "127.0.0.1")
# port of the netapp's server
NETAPP_PORT = os.getenv("NETAPP_PORT", 5896)
# test video file
TEST_VIDEO_FILE = os.getenv("TEST_VIDEO_FILE", "../../../../assets/2017_01_02_001021_s.mp4")


class ResultsViewer(Thread):

    def __init__(self, **kw):
        super().__init__(**kw)
        self.stop_event = Event()

    def stop(self):
        self.stop_event.set()

    def run(self):
        logging.info("Thread %s: starting", self.name)
        while not self.stop_event.is_set():
            if not results_storage.empty():
                results = results_storage.get(timeout=1)
                timestamp = int(results['timestamp'])
                if DEBUG_PRINT_DELAY:
                    time_now = math.floor(time.time() * 100)
                    print(f"{(time_now - timestamp) / 100.}s delay")
                try:
                    frame = image_storage.pop(timestamp)
                    detections = results["detections"]
                    for d in detections:
                        score = d["score"]
                        if DEBUG_PRINT_SCORE and score > 0:
                            print(score)
                        cls_name = d["class_name"]
                        # Draw detection into frame.
                        x1, y1, x2, y2 = [int(coord) for coord in d["bbox"]]
                        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 1)
                        font = cv2.FONT_HERSHEY_SIMPLEX
                        cv2.putText(frame, f"{cls_name} ({score * 100:.0f})%", (x1, y1 - 5), font, .5, (0, 255, 0), 1,
                                    cv2.LINE_AA)
                    try:
                        cv2.imshow("Results", frame)
                        cv2.waitKey(1)
                    except Exception as ex:
                        print(ex)
                    results_storage.task_done()
                except KeyError as ex:
                    print(ex)


def get_results(results: str):
    """
    Callback which process the results from the NetApp

    Args:
        results (str): The results in json format
    """

    print(results)
    if 'timestamp' in results:
        results_storage.put(results, block=False)
    pass


def main():
    """
    Creates the client class and starts the data transfer
    """

    results_viewer = ResultsViewer(name="test_client_http_viewer", daemon=True)
    results_viewer.start()

    client = None
    global stopped
    stopped = False

    def signal_handler(sig, frame):
        global stopped
        stopped = True
        results_viewer.stop()
        print(f"Terminating ({signal.Signals(sig).name})...")
    signal.signal(signal.SIGTERM, signal_handler)
    signal.signal(signal.SIGINT, signal_handler)

    try:
        # creates the NetApp client with gstreamer extension
        client = NetAppClient(None, None, None, None, True, get_results, False, False, NETAPP_ADDRESS, NETAPP_PORT)
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
            image_storage[timestamp] = resized
            client.send_image(resized, str(timestamp), 5)

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


if __name__ == '__main__':
    main()
