from threading import Thread, Event
from .data_sender_gstreamer import DataSenderGStreamer
import cv2


class DataSenderGStreamerFromSource(Thread):
    """
    Class which setups gstreamer connection to the NetApp and
    sends the data from the gstreamer source.
    """

    def __init__(self, ip: str, port: int, data_source: str, fps: float, width: int, height: int, resize: bool = True,
                 threads: int = 1, **kw):
        """
        Constructor

        Args:
            ip (str): ip address or hostname of the NetApp interface
            port (int): the port assigned for gstreamer communication
            data_source (str): the gstreamer source, e.g. v4l2src device=/dev/video0
            fps (float): the requested FPS of the h264 stream
            width (int): the width (in pixels) each frame of the stream should be resized to 
                (if resize=True) or the actual width of the frames in stream (if resize=False)
            height (int): the height (in pixels) the frame of the stream should be resized to 
                (if resize=True) or the actual height of the frames in stream (if resize=False)
            resize (bool): indicates if the frames in the stream should be resized before sending. 
                If False, the width and height parameters must contain the actual resolution of
                the frames in the stream
            threads (int, optional): the number of threads to be used to encode the h264 stream. 
                Defaults to 1.
        """

        super().__init__(**kw)
        self.stop_event = Event()
        # instantiate the base data sender object
        self.data_sender_gstreamer = DataSenderGStreamer(ip, port, fps, width, height)

        # creates the video capture and runs the thread
        self.cap = cv2.VideoCapture(data_source, cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            raise Exception("Cannot open video stream")

        self.resize = resize

    def stop(self):
        self.stop_event.set()

    def run(self):
        """
        Reads the data from the gstreamer source and sends it using the base data sender
        """

        while not self.stop_event.is_set():
            ret, frame = self.cap.read()
            if not ret:
                break
            if self.resize:
                resized = cv2.resize(frame, (self.data_sender_gstreamer.width, self.data_sender_gstreamer.height),
                                     interpolation=cv2.INTER_AREA)
                self.data_sender_gstreamer.send_image(resized)
            else:
                self.data_sender_gstreamer.send_image(frame)