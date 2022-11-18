import cv2
import requests
import socketio
from collections.abc import Callable
import numpy as np

class FailedToConnect(Exception):
    """
    Exception which is raised when the client could not connect to the NetApp.
    """
    pass


class NetAppClient():
    """
    Basic implementation of the NetApp client. It creates the Requests 
    object with session and bind callbacks for connection info 
    and results from the NetApp.
    """

    def __init__(self, host: str, port: int, results_event: Callable) -> None:
        """
        Constructor

        Args:
            host (str): IP address or hostname of the NetApp interface
            port (int): port of the NetApp interface
            results_event (Callable): callback where results will arrive
        """
        self.sio = socketio.Client()
        self.s = requests.session()
        self.host = host.rstrip('/')
        self.port = port
        self.sio.on("message", results_event, namespace='/results')
        self.sio.on("connect", self.on_connect_event, namespace='/results')
        self.sesion_cookie = None
        
    def register(self, args=dict()) -> str:
        """
        Calls the /register endpoint of the NetApp interface and if the 
        registration is successfull, it setups the websocket connection
        for results retrieval.

        Args:
            args (_type_, optional): optional parameters to be passed to 
            the NetApp. Defaults to dict().

        Returns:
            str: response from the NetApp
        """
        resp = self.s.get(self.build_api_endpoint("register"), params=args)
        self.session_cookie = resp.cookies["session"]

        # creates the websocket connection
        self.sio.connect(self.build_api_endpoint(""), namespaces=['/results'], headers={'Cookie': f'session={self.session_cookie}'})
        return resp        

    def disconnect(self):
        """
        Calls the /unregister endpoint of the server and disconnects the 
        websocket connection.
        """
        self.s.get(self.build_api_endpoint("unregister"))
        self.sio.disconnect()
 
    def build_api_endpoint(self, path: str):
        """
        Builds and API endpoint on the NetApp interface.

        Args:
            path (str): endpoint path

        Returns:
            _type_: complete URI to requested endpoint
        """
        return f"http://{self.host}:{self.port}/{path}"

    def wait(self):
        """
        Blocking infinite waiting.
        """
        self.sio.wait()

    def on_connect_event(self):
        """
        Callback called once the connection to the NetApp is made.
        """
        print("Connected to server")

    def send_image(self, frame: np.ndarray, timestamp:str = None):
        """
        Encodes the received image frame to the jpg format and sends
        it over the HTTP, to the /image endpoint.

        Args:
            frame (np.ndarray): image frame
            timestamp (str, optional): frame timestamp The timestamp format
            is defined by the NetApp. Defaults to None.
        """
        _, img_encoded = cv2.imencode('.jpg', frame)
        headers = {'content-type': 'image/jpeg'}
        self.s.post(self.build_api_endpoint("image"), data=img_encoded.tostring(), 
            headers=headers, params={"timestamp": timestamp})


