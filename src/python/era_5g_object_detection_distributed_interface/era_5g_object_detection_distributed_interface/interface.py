import argparse
import logging
import os
import sys
import time
from functools import partial
from queue import Queue
from typing import Dict, Optional, Tuple, Any

from era_5g_object_detection_distributed_interface.results_reader import ResultsReader
from era_5g_object_detection_distributed_interface.task_handler_distributed import TaskHandlerCelery

from era_5g_interface.channels import CallbackInfoServer, ChannelType, DATA_NAMESPACE, DATA_ERROR_EVENT
from era_5g_interface.dataclasses.control_command import ControlCommand, ControlCmdType
from era_5g_interface.interface_helpers import HeartbeatSender, NETAPP_STATUS_ADDRESS

from era_5g_server.server import NetworkApplicationServer


logging.basicConfig(stream=sys.stdout, level=logging.DEBUG, format="%(asctime)s [%(levelname)s] %(name)s: %(message)s")
logger = logging.getLogger("Distributed 5G-ERA Network Application interface")

# Port of the 5G-ERA Network Application's server
NETAPP_PORT = os.getenv("NETAPP_PORT", 5896)



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
                "image_h264": CallbackInfoServer(ChannelType.H264, self.image_callback),
                "json": CallbackInfoServer(ChannelType.JSON, self.json_callback),
            },
            *args,
            **kwargs,
        )

        # Registered tasks and results readers
        self.tasks = dict()
        self.results_readers = dict()

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

        task_handler = self.tasks[eio_sid]
        task_handler.store_image({"timestamp": data["timestamp"], "recv_timestamp": time.perf_counter_ns()}, data["frame"])

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

            # Queue for passing info about celery jobs (for results retrieval)
            jobs_info_queue = Queue(1024)

            task_handler = TaskHandlerCelery(jobs_info_queue)
            
            # Create a results reader, which periodically reads status of jobs
            # to find finished jobs and pass the results to the robot
            send_function = partial(self.send_data, event="results", sid=self.get_sid_of_data(eio_sid))
            results_reader = ResultsReader(jobs_info_queue, send_function, name=f"results_reader_{eio_sid}", daemon=True)
            results_reader.start()

            self.results_readers[eio_sid] = results_reader
            self.tasks[eio_sid] = task_handler

            logger.info(f"Client registered: {eio_sid}")

        logger.info(
            f"Control command applied, eio_sid {eio_sid}, sid {sid}, "
            f"results sid {self.get_sid_of_data(eio_sid)}, command {command}"
        )
        return True, (
            f"Control command applied, eio_sid {eio_sid}, sid {sid}, results sid"
            f" {self.get_sid_of_data(eio_sid)}, command {command}"
        )

    def disconnect_callback(self, sid: str) -> None:
        """Called with client disconnection

        Args:
            sid (str): Namespace sid.
        """

        eio_sid = self.get_eio_sid_of_data(sid)
        self.tasks.pop(eio_sid)
        results_reader = self.results_readers.pop(eio_sid)
        results_reader.stop()

        logger.info(f"Client disconnected from {DATA_NAMESPACE} namespace: session id: {sid}")


def main(args=None):
    logging.getLogger().setLevel(logging.INFO)

    server = Server(port=NETAPP_PORT, host="0.0.0.0")

    try:
        server.run_server()
    except KeyboardInterrupt:
        logger.info("Terminating ...")


if __name__ == '__main__':
    main()
