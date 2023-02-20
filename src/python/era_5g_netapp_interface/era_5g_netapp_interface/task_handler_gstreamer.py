from abc import ABC
import logging
import cv2

from era_5g_netapp_interface.task_handler import TaskHandler, TaskHandlerInitializationFailed


class TaskHandlerGstreamer(TaskHandler, ABC):
    """
    Abstract class. Task handler which takes care of reading the data 
    from Gstreamer pipeline with defined parameters. It needs to be 
    inherited to implement the store_image method.
    """

    def __init__(self, sid: str, port: int, **kw):
        """
        Constructor

        Args:
            sid (str): The session id obtained from NetApp client. It is used to 
                match the results with the data sender.
            port (int): The port where the Gstreamer pipeline should listen to.
        """

        super().__init__(sid=sid, **kw)
        self.port = port

    def run(self):
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
            logging.info(f"Creating Gstreamer capture on port {self.port}")
            # standard OpenCV VideoCapture connected to the Gstreamer pipeline
            cap = cv2.VideoCapture(camSet)
            if not cap.isOpened():
                raise TaskHandlerInitializationFailed("VideoCapture was not opened")
            logging.info("Gstreamer capture created")
        except:
            logging.info("Gstreamer capture fail")
            exit(1)

        while not self.stop_event.is_set():
            ret, frame = cap.read()
            if ret:
                # extract the timestamp from the frame
                timestamp = cap.get(cv2.CAP_PROP_POS_MSEC)
                self.store_image({"sid": self.sid, "websocket_id": self.websocket_id, "timestamp": timestamp}, frame)
