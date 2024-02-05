import argparse
import logging
import os
import sys
import time
import traceback
from queue import Queue
from typing import Dict, Optional, Tuple, Any

import numpy as np

from era_5g_interface.channels import CallbackInfoServer, ChannelType, DATA_NAMESPACE, DATA_ERROR_EVENT
from era_5g_interface.dataclasses.control_command import ControlCommand, ControlCmdType
from era_5g_interface.interface_helpers import HeartBeatSender, MIDDLEWARE_REPORT_INTERVAL, RepeatedTimer
from era_5g_interface.task_handler_internal_q import TaskHandlerInternalQ
from era_5g_object_detection_common.image_detector import ImageDetector, ImageDetectorInitializationFailed
from era_5g_object_detection_standalone.worker_face import FaceDetectorWorker
from era_5g_object_detection_standalone.worker_fps import FpsDetectorWorker
from era_5g_object_detection_standalone.worker_mmdet import MMDetectorWorker
from era_5g_server.server import NetworkApplicationServer

logging.basicConfig(stream=sys.stdout, level=logging.DEBUG, format="%(asctime)s [%(levelname)s] %(name)s: %(message)s")
logger = logging.getLogger("Standalone 5G-ERA Network Application interface")

# Port of the 5G-ERA Network Application's server.
NETAPP_PORT = os.getenv("NETAPP_PORT", 5896)
# Input queue size.
NETAPP_INPUT_QUEUE = int(os.getenv("NETAPP_INPUT_QUEUE", 1))

DetectorWorker: Optional[type] = None


class Server(NetworkApplicationServer):
    """Server receives images from clients, manages tasks and workers, sends results to clients."""

    def __init__(
        self,
        *args,
        **kwargs,
    ) -> None:
        """Constructor.

        Args:
            *args: NetworkApplicationServer arguments.
            **kwargs: NetworkApplicationServer arguments.
        """

        super().__init__(
            callbacks_info={
                "image": CallbackInfoServer(ChannelType.JPEG, self.image_callback),
                "json": CallbackInfoServer(ChannelType.JSON, self.json_callback),
            },
            *args,
            **kwargs,
        )

        # The image detector to be used.
        self.detector_threads: Dict[str, ImageDetector] = {}

        # Dict of registered tasks.
        self.tasks: Dict[str, TaskHandlerInternalQ] = {}

        self.heart_beat_sender = HeartBeatSender()
        heart_beat_timer = RepeatedTimer(MIDDLEWARE_REPORT_INTERVAL, self.heart_beat)
        heart_beat_timer.start()

    def heart_beat(self):
        """Heart beat generation and sending."""

        latencies = []
        for worker in self.detector_threads.values():
            latencies.extend(worker.latency_measurements.get_latencies())
        avg_latency = 0
        if len(latencies) > 0:
            avg_latency = float(np.mean(np.array(latencies)))

        queue_size = NETAPP_INPUT_QUEUE
        queue_occupancy = 1  # TODO: Compute for every worker?

        self.heart_beat_sender.send_application_heart_beat(
            avg_latency=avg_latency,
            queue_size=queue_size,
            queue_occupancy=queue_occupancy,
            current_robot_count=len(self.tasks),
        )

    def image_callback(self, sid: str, data: Dict[str, Any]):
        """Allows to receive decoded image using the websocket transport.

        Args:
            sid (str): Namespace sid.
            data (Dict[str, Any]): Data dict including decoded frame (data["frame"]) and send timestamp
                (data["timestamp"]).
        """

        eio_sid = self._sio.manager.eio_sid_from_sid(sid, DATA_NAMESPACE)

        if eio_sid not in self.tasks:
            logger.error("Non-registered client tried to send data")
            self.send_data({"message": "Non-registered client tried to send data"}, DATA_ERROR_EVENT, sid=sid)
            return

        task = self.tasks[eio_sid]
        task.store_data({"timestamp": data["timestamp"], "recv_timestamp": time.perf_counter_ns()}, data["frame"])

    def json_callback(self, sid: str, data: Dict):
        """Allows to receive general json data using the websocket transport.

        Args:
            sid (str): Namespace sid.
            data (Dict): 5G-ERA Network Application specific JSON data.
        """

        logger.info(f"Client with task id: {self.get_eio_sid_of_data(sid)} sent data {data}")

    def command_callback(self, command: ControlCommand, sid: str) -> Tuple[bool, str]:
        """Receive and process Control Commands.

        Args:
            command (ControlCommand): Control command to be processed.
            sid (str): Namespace sid.

        Returns:
            (initialized (bool), message (str)) or initialized (bool): If False, initialization failed.
        """

        eio_sid = self.get_eio_sid_of_control(sid)

        logger.info(f"Control command {command} processing: session id: {sid}")

        if command and command.cmd_type == ControlCmdType.INIT:
            # Check that initialization has not been called before.
            if eio_sid in self.tasks:
                logger.error(f"Client attempted to call initialization multiple times.")
                self.send_command_error("Initialization has already been called before", sid)
                return False, "Initialization has already been called before"

            # Queue with received images.
            image_queue = Queue(NETAPP_INPUT_QUEUE)

            task = TaskHandlerInternalQ(image_queue)

            try:
                detector = DetectorWorker(
                    image_queue,
                    lambda results: self.send_data(data=results, event="results", sid=self.get_sid_of_data(eio_sid)),
                    name=f"Detector {eio_sid}",
                    daemon=True,
                )
            except Exception as ex:
                logger.error(f"Failed to create Detector: {repr(ex)}")
                logger.error(traceback.format_exc())
                self.send_command_error(f"Failed to create Detector: {repr(ex)}", sid)
                return False, f"Failed to create Detector: {repr(ex)}"

            self.tasks[eio_sid] = task
            self.detector_threads[eio_sid] = detector
            self.detector_threads[eio_sid].start()
            t0 = time.perf_counter_ns()
            while True:
                if self.detector_threads[eio_sid].is_alive():
                    break
                if time.perf_counter_ns() > t0 + 5 * 1.0e9:
                    logger.error(f"Timed out to start worker. Session id: {eio_sid}, namespace_id: {sid}")
                    return False, f"Timed out to start worker"

            logger.info(f"Task handler and worker created and started: {eio_sid}")

        logger.info(
            f"Control command applied, eio_sid {eio_sid}, sid {sid}, "
            f"results sid {self.get_sid_of_data(eio_sid)}, command {command}"
        )
        return True, (
            f"Control command applied, eio_sid {eio_sid}, sid {sid}, results sid"
            f" {self.get_sid_of_data(eio_sid)}, command {command}"
        )

    def disconnect_callback(self, sid: str) -> None:
        """Called with client disconnection - deletes task and worker.

        Args:
            sid (str): Namespace sid.
        """

        eio_sid = self.get_eio_sid_of_data(sid)
        self.tasks.pop(eio_sid)
        detector = self.detector_threads.pop(eio_sid)
        detector.stop()
        logger.info(f"Client disconnected from {DATA_NAMESPACE} namespace: session id: {sid}")


def main():
    """Main function."""

    parser = argparse.ArgumentParser(description="Standalone variant of object detection 5G-ERA Network Application")
    parser.add_argument(
        "--detector",
        default="mmdetection",
        help="Select detector. Available options are opencv, mmdetection, fps. Default is mmdetection.",
    )

    args = parser.parse_args()
    global DetectorWorker

    # Creates detector and runs it as thread, listening to image_queue
    try:
        if args.detector == "fps":
            DetectorWorker = FpsDetectorWorker
        elif args.detector == "mmdetection":
            DetectorWorker = MMDetectorWorker
        elif args.detector == "opencv":
            DetectorWorker = FaceDetectorWorker
        else:
            raise ImageDetectorInitializationFailed(
                "Invalid detector selected. Available options are opencv, mmdetection, fps."
            )

    except ImageDetectorInitializationFailed as ex:
        logger.error(ex)
        exit()

    logger.info(f"The size of the queue set to: {NETAPP_INPUT_QUEUE}")

    # runs the flask server
    # allow_unsafe_werkzeug needs to be true to run inside the docker
    # TODO: use better webserver
    server = Server(port=NETAPP_PORT, host="0.0.0.0")

    try:
        server.run_server()
    except KeyboardInterrupt:
        logger.info("Terminating ...")


if __name__ == "__main__":
    main()
