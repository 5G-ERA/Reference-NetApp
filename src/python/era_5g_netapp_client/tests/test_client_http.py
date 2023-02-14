from era_5g_netapp_client.data_sender_gstreamer_from_source import DataSenderGStreamerFromSource
from era_5g_netapp_client.data_sender_gstreamer_from_file import DataSenderGStreamerFromFile
from era_5g_netapp_client.client import FailedToConnect, NetAppClient
import traceback
import math
import time
import cv2

from queue import Queue

from era_5g_netapp_interface.common import ThreadBase, get_logger
import logging

image_storage = dict()
results_storage = Queue()

DEBUG_PRINT_SCORE = False  # useful for FPS detector
DEBUG_PRINT_DELAY = False  # prints the delay between capturing image and receiving the results
FROM_SOURCE = False  # Video from source

logger = get_logger(log_level=logging.INFO)


class ResultsViewer(ThreadBase):

    def __init__(self, logger, name):
        super().__init__(logger, name)
        self.stopped = False

    def run(self):
        while not self.stopped:
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

    # ip address or hostname of the computer, where the netapp is deployed
    netapp_uri = "127.0.0.1"
    # port of the netapp's server
    netapp_port = 5897
    results_viewer = ResultsViewer(logger, "viewer")
    results_viewer.start(daemon=False)
    # to avoid exception in "except"
    client = None

    try:
        # creates the NetApp client with gstreamer extension
        client = NetAppClient(None, None, None, None, "True", get_results, False, False, netapp_uri, netapp_port)
        # register the client with the NetApp
        client.register()
        if FROM_SOURCE:
            # creates a video capture to pass images to the NetApp either from webcam ...
            cap = cv2.VideoCapture(0)
            if not cap.isOpened():
                raise Exception("Cannot open camera")
        else:
            # or from video file
            cap = cv2.VideoCapture("../../../../assets/2017_01_02_001021_s.mp4")
            if not cap.isOpened():
                raise Exception("Cannot open video file")

        while True:
            ret, frame = cap.read()
            timestamp = math.floor(time.time() * 100)
            if not ret:
                break
            resized = cv2.resize(frame, (640, 480), interpolation=cv2.INTER_AREA)

            image_storage[timestamp] = resized
            client.send_image(resized, str(timestamp), 5)

    except FailedToConnect as ex:
        print(f"Failed to connect to server ({ex})")
        results_viewer.stop()
    except KeyboardInterrupt:
        print("Terminating...")
        results_viewer.stop()
    except Exception as ex:
        traceback.print_exc()
        print(f"Failed to create client instance ({ex})")
        results_viewer.stop()
    finally:
        if client is not None:
            client.disconnect()


if __name__ == '__main__':
    main()
