
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
    server_ip = "butcluster.ddns.net"  
    user = "2c63955e-27cf-4b9a-980b-b70a54281d15"
    password = "test"
    #server_ip = "172.17.0.2"    

    try:
        # to avoid exception in "except"
        client = None
        # creates the NetApp client with gstreamer extension
        client = NetAppClientGstreamer(server_ip, user, password, "7d93728a-4a4c-4dae-8245-16b86f85b246", True, get_results, True, True)
        #client.wait_until_netapp_ready()
        #client = NetAppClientGstreamer("", user, password, "7d93728a-4a4c-4dae-8245-16b86f85b246", True, get_results, False, False, "192.168.206.11", 5896)
        print("resource should be ready now")
        client.register()
        # register the client with the NetApp
        #client.register()
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
