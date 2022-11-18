
from era_5g_netapp_client.data_sender_gstreamer_from_source import DataSenderGStreamerFromSource 
from era_5g_netapp_client.client_gstreamer import NetAppClientGstreamer
from era_5g_netapp_client.client import FailedToConnect

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
    server_ip = "172.17.0.2"    

    try:
        # creates the NetApp client with gstreamer extension
        client = NetAppClientGstreamer(server_ip, 5896, get_results)
        # register the client with the NetApp
        client.register()
        # creates a data sender which will pass images from webcamera to the NetApp
        sender = DataSenderGStreamerFromSource(server_ip, client.gstreamer_port, "v4l2src device=/dev/video0", 15)
        # waits infinitely
        client.wait()
    except FailedToConnect as ex:
        print(f"Failed to connect to server ({ex})")
    except KeyboardInterrupt:
        client.disconnect()   

if __name__ == '__main__':
    main()
