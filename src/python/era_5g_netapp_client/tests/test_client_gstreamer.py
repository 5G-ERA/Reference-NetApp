
from era_5g_netapp_client.data_sender_gstreamer_from_source import DataSenderGStreamerFromSource 
from era_5g_netapp_client.client_gstreamer import NetAppClientGstreamer
from era_5g_netapp_client.client import FailedToConnect
import traceback
import time

def get_results(results: str):
    """
    Callback which process the results from the NetApp

    Args:
        results (str): The results in json format
    """
    print(results)
    pass

def main():
    """
    Creates the client class and starts the data transfer
    """

    # IP address or hostname of the server
    server_ip = "IP_OR_HOSTNAME"
    user = "GUID"
    password = "passwd"
    
    # if middleware is ommited, the netapp address and port has to be specified manually
    #netapp_uri = "localhost"
    #netapp_port = "5896"  

    try:
        # to avoid exception in "except"
        client = None
        # creates the NetApp client with gstreamer extension
        client = NetAppClientGstreamer(server_ip, user, password, "7d93728a-4a4c-4dae-8245-16b86f85b246", True, get_results, True, True)
        # register the client with the NetApp
        client.register()
        # creates a data sender which will pass images from webcamera to the NetApp
        sender = DataSenderGStreamerFromSource(client.netapp_host, client.gstreamer_port, "v4l2src device=/dev/video0", 15)
        # waits infinitely
        client.wait()
    except FailedToConnect as ex:
        print(f"Failed to connect to server ({ex})")
    except KeyboardInterrupt as ex:
        print("Terminating...")
    except Exception as ex:
        traceback.print_exc()
        print(f"Failed to create client instance ({ex})") 
    finally:
        if client is not None:
            client.disconnect() 


if __name__ == '__main__':
    main()
