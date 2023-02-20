import datetime
import os
from threading import Thread
import logging
from logging import Logger
from pkg_resources import ensure_directory
from typing import Union


def get_logger(log_level: Union[int, str], logdir: str = None, deploy: bool = False) -> Logger:
    """
    Gets logger for thread-based classes.

    Args:
        log_level (Union[int, str]): Sets the threshold for this logger to level.
            Logging messages which are less severe than level will be ignored.
            Possible levels are CRITICAL, ERROR, WARNING, INFO, DEBUG, NOTSET
        logdir (str, optional): If set, the logs will be saved to this directory.
            Defaults to None.
        deploy (bool, optional): TBA. Defaults to False.

    Returns:
        _type_: _description_
    """
    logger = logging.getLogger()
    logger.setLevel(log_level)
    formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
    if not deploy:
        # Log to console
        handler = logging.StreamHandler()
        handler.setLevel(log_level)
        handler.setFormatter(formatter)
        logger.addHandler(handler)
    # Log to file
    if logdir is not None:
        ensure_directory(logdir)
        log_filepath = os.path.join(logdir, "%s.log" % datetime.datetime.utcnow())
        file_handler = logging.FileHandler(log_filepath)
        file_handler.setLevel(log_level)
        file_handler.setFormatter(formatter)
        logger.addHandler(file_handler)

    return logger


class ThreadBase(Thread):
    """
    Base Thread class which provides handy methods.
    """

    def __init__(self, logger: Logger, name: str = None):
        """
        Constructor

        Args:
            logger (Logger): A thread-safe logger. Could be obtained using the
                era_5g_netapp_interface.common.get_logger() function.
            name (str): Name of the thread
        """

        super().__init__()
        self._monitoring = None
        self._print = False
        self.stopped = False
        self._stopped = False
        self.logger = logger
        self.name = str(name or super().getName())

    def send_state(self):
        if self._monitoring:
            self._send_state()
        return

    def _send_state(self):
        pass

    def set_monitoring(self, _monitoring):
        self._monitoring = _monitoring
        return

    def set_print(self, _print):
        self._print = _print
        return

    def stop(self):
        self.logger.debug("Stopping %s thread", self.name)
        self.stopped = True

    def stop_base(self):
        self.logger.debug("Stopping %s thread", self.name)
        self._stopped = True

    def start(self, daemon: bool):
        self.logger.debug("Starting %s thread", self.name)
        t = Thread(target=self.run, args=())
        t.daemon = daemon
        t.start()

    def run(self):
        self._run()

    def _run(self):
        pass
