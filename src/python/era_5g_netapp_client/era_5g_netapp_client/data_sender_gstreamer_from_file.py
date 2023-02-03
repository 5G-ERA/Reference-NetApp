from threading import Thread
from .data_sender_gstreamer import DataSenderGStreamer
import cv2


class DataSenderGStreamerFromFile(Thread):
    """
    Class which setups gstreamer connection to the NetApp and
    sends the data from the gstreamer source.

    """

    def __init__(self, ip: str, port: int, fps: float, file_name: str, width: int, height: int, resize:bool = True, threads:int = 1):
        """
        Constructor

        Args:
            ip (str): ip address or hostname of the NetApp interface
            port (int): the port assigned for gstreamer communication
            data_source (str): the gstreamer source, e.g. v4l2src device=/dev/video0
            fps (float): the requested FPS of the h264 stream
            width (int): the width (in pixels) the video should be resized to (if resize=True) or the
                actual width of the video (if resize=False)
            height (int): the height (in pixels) the video should be resized to (if resize=True) or the
                actual height of the video (if resize=False)
            resize (bool): indicates if the video should be resized before sending. If False, 
                the width and heigth parameters must contain the actual resolution of the video
            threads (int, optional): the number of threads to be used to encode the h264 stream. 
                Defaults to 1.
        """

        # instantiate the base data sender object
        self.data_sender_gstreamer = DataSenderGStreamer(ip, port, fps, width, height)
        
        # creates the videcapture and runs the thread
        self.cap = cv2.VideoCapture(file_name)
        self.alive = True
        self.resize = resize
        t = Thread(target=self.run, args=())
        t.daemon = True
        t.start()

    def run(self):
        """
        Reads the data from the gstreamer source and sends it using the base data sender
        """
        while True:
            ret, frame = self.cap.read()
            if self.resize:
                resized = cv2.resize(frame, (self.data_sender_gstreamer.width, self.data_sender_gstreamer.height), 
                                     interpolation = cv2.INTER_AREA)
            if ret == False:
                break
            if self.resize:
                self.data_sender_gstreamer.send_image(resized)
            else:
                self.data_sender_gstreamer.send_image(frame)
            
                
        self.alive = False