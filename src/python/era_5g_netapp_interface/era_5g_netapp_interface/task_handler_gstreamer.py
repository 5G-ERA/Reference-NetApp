from abc import ABC
from logging import Logger
import cv2

from era_5g_netapp_interface.task_handler import TaskHandler, TaskHandlerInitializationFailed


class TaskHandlerGstreamer(TaskHandler, ABC):
    """
    Abstract class. Task handler which takes care of reading the data 
    from Gstreamer pipeline with defined parameters. It needs to be 
    inherited to implement the store_image method.
    """

    def __init__(self, logger: Logger, sid: str, port: str, **kwargs):
        """
        Constructor

        Args:
            logger (Logger): A thread-safe logger. Could be obtained using the
                era_5g_netapp_interface.common.get_logger() function.
            sid (str): The session id obtained from NetApp client. It is used to 
                match the results with the data sender.
            port (str): The port where the Gstreamer pipeline should listen to.
        """

        super().__init__(logger=logger, sid=sid, **kwargs)
        self.port = port

    def _run(self):
        """
        The infinite loop which reads the Gstreamer pipeline and pass the images
        to the store_image method, which has to be implemented in the child class.

        Raises:
            TaskHandlerInitializationFailed: Raised when the construction of
                Gstreamer pipeline failed
        """

        # pipeline which decodes a h264 stream 
        camSet = f'udpsrc port={self.port} caps="application/x-rtp,media=(string)video,encoding-name=(string)H264,' \
                 f'payload=(int)96" ! rtpjitterbuffer ! rtph264depay ! avdec_h264 ! videorate ! videoconvert ! ' \
                 f'appsink'

        try:
            self.logger.info(f"creating capture on port {self.port}")
            # standard OpenCV VideoCapture connected to the Gstreamer pipeline
            cap = cv2.VideoCapture(camSet)
            if not cap.isOpened():
                raise TaskHandlerInitializationFailed("Videocapture was not opened")
            self.logger.info("capture created")
        except:
            self.logger.info("capture fail")
            exit(1)

        while not self.stopped:
            ret, frame = cap.read()
            if ret:
                # extract the timestamp from the frame
                timestamp = cap.get(cv2.CAP_PROP_POS_MSEC)
                self.store_image({"sid": self.sid, "websocket_id": self.websocket_id, "timestamp": timestamp}, frame)
