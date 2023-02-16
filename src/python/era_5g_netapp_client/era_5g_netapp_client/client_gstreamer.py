from typing import Callable
from .client import NetAppClient
from .client import FailedToConnect


class NetAppClientGstreamer(NetAppClient):
    """
    NetApp client which asks the NetApp for h264 stream setup.

    Args:
        NetAppClient (_type_): the base client
    """

    def __init__(self, host: str, user_id: str, password: str, task_id: str, resource_lock: bool,
                 results_event: Callable, use_middleware: bool = True, wait_for_netapp: bool = True,
                 netapp_uri: str = None, netapp_port: int = None) -> None:
        """
        Constructor

        Args:
            host (str): The IP or hostname of the middleware
            user_id (str): The middleware user's id
            password (str): The middleware user's password
            task_id (str): The ID of the NetApp to be deployed
            resource_lock (bool): TBA
            results_event (Callable): callback where results will arrive
            use_middleware (bool, optional): Defines if the NetApp should be deployed by middleware.
                If False, the netapp_uri and netapp_port need to be defined, otherwise,
                they need to be left None. Defaults to True.
            wait_for_netapp (bool, optional):
            netapp_uri (str, optional): The URI of the NetApp interface. Defaults to None.
            netapp_port (int, optional): The port of the NetApp interface. Defaults to None.

        """
        super().__init__(host, user_id, password, task_id, resource_lock, results_event, use_middleware,
                         wait_for_netapp, netapp_uri, netapp_port)
        # TODO: update args
        # holds the gstreamer port
        self.gstreamer_port = None

    def register(self, args=None) -> str:
        """
        Calls the /register endpoint of the NetApp interface and if the
        registration is successful, it sets up the websocket connection
        for results retrieval. Besides, it obtains the gstreamer port.

        Args:
            args (_type_, optional): optional parameters to be passed to
                the NetApp, in the form of dict. Defaults to None.

        Raises:
            FailedToConnect: raised when connection failed or the server
                responded with wrong data

        Returns:
            str: response from the NetApp
        """

        if args is None:
            merged_args = {"gstreamer": True}
        else:
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
