import cv2


class DataSenderGStreamer():
    """
    Class which setups gstreamer connection to the NetApp allowing
    to send image frames using the OpenCV VideoWriter.
    """

    def __init__(self, host: str, port: int, fps: float, width: int, height: int, threads: int = 1):
        """
        Constructor

        Args:
            host (str): ip address or hostname of the NetApp interface
            port (int): the port assigned for gstreamer communication
            fps (float): the requested FPS of the h264 stream
            threads (int): the number of threads to be used to encode the h264 stream. 
                Defaults to 1
        """
        
        self.host = host
        self.port = port
        self.fps = fps
        self.threads = threads
        self.width = width
        self.height = height
        
        # default pipeline for sending h264 encoded stream
        # ultrafast and zerolatency params for near real-time processing
        gst_str_rtp = 'appsrc ! videoconvert ! queue ! x264enc ' + \
            'speed-preset=ultrafast  tune=zerolatency  byte-stream=true ' + \
            f'threads={self.threads} key-int-max=15 intra-refresh=true ! h264parse ! ' + \
            f'rtph264pay ! queue ! udpsink host={self.host} port={self.port}'
        self.out = cv2.VideoWriter(gst_str_rtp, cv2.CAP_GSTREAMER, 0, fps, (width, height), True)

    def send_image(self, frame):
        self.out.write(frame)