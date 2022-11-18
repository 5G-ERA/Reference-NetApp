from threading import Thread
from .data_sender_gstreamer import DataSenderGStreamer
import cv2


class DataSenderGStreamerFromSource(Thread):
    """
    Class which setups gstreamer connection to the NetApp and
    sends the data from the gstreamer source.

    """

    def __init__(self, ip: str, port: int, data_source: str, fps: float, threads:int = 1):
        """
        Constructor

        Args:
            ip (str): ip address or hostname of the NetApp interface
            port (int): the port assigned for gstreamer communication
            data_source (str): the gstreamer source, e.g. v4l2src device=/dev/video0
            fps (float): the requested FPS of the h264 stream
            threads (int, optional): the number of threads to be used to encode the h264 stream. 
                Defaults to 1.
        """

        # instantiate the base data sender object
        self.data_sender_gstreamer = DataSenderGStreamer(ip, port, fps)
        
        # TODO: make the resolution of the image parameterizable
        gst_str = f"{data_source} ! video/x-raw, format=YUY2, width=640, height=480, " + \
            "pixel-aspect-ratio=1/1 ! videoconvert ! appsink"

        # creates the videcapture and runs the thread
        self.cap = cv2.VideoCapture(gst_str,cv2.CAP_GSTREAMER)
        t = Thread(target=self.run, args=())
        t.daemon = True
        t.start()

    def run(self):
        """
        Reads the data from the gstreamer source and sends it using the base data sender
        """
        while True:
            ret, frame = self.cap.read()
            if ret == False:
                break
            self.data_sender_gstreamer.send_image(frame)