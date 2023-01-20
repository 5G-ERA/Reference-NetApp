from typing import Callable
from .client import NetAppClient
from .client import FailedToConnect

class NetAppClientGstreamer(NetAppClient):
    """
    NetApp client which asks the NetApp for h264 stream setup.

    Args:
        NetAppClient (_type_): the base client
    """

    def __init__(self, host: str, user_id: str, password: str, task_id: str, resource_lock: str, results_event: Callable, use_middleware=True, wait_for_netapp=True, netapp_uri: str = None, netapp_port: int = None) -> None:
        super().__init__(host, user_id, password, task_id, resource_lock, results_event, use_middleware, wait_for_netapp, netapp_uri, netapp_port)
        """
        Constructor

        Args:
            host (str): IP address or hostname of the NetApp interface
            port (int): port of the NetApp interface
            results_event (Callable): callback where results will arrive
        """
        # TODO: update args
        # holds the gstreamer port
        self.gstreamer_port = None


    def register(self, args=dict()) -> str:
        """
        Calls the /register endpoint of the NetApp interface and if the 
        registration is successfull, it setups the websocket connection
        for results retrieval. Besides, it obtains the gstreamer port.

        Args:
            args (_type_, optional): optional parameters to be passed to 
            the NetApp. Defaults to dict().

        Raises:
            FailedToConnect: raised when connection failed or the server
                responsed with wrong data

        Returns:
            str: response from the NetApp
        """
        

        merged_args = {**args, **{"gstreamer": True}}
        resp = super().register(merged_args)

        # checks whether the NetApp responded with any data
        if len(resp.content) > 0:
            data = resp.json()
            # checks if an error was returned
            if "error" in data:
                err = data["error"]
                self.disconnect()
                raise FailedToConnect(f"{resp.status_code}: {err}")
            # checks if gstreamer port was returned
            if "port" in data:                
                self.gstreamer_port = data["port"]
            else:
                raise FailedToConnect(f"{resp.status_code}: could not obtain the gstreamer port number")
        
        else:
            self.disconnect()
            raise FailedToConnect(f"{resp.status_code}: unknown error")

        return resp