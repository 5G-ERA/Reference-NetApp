from era_5g_netapp_client.data_sender_gstreamer_from_source import DataSenderGStreamerFromSource
from era_5g_netapp_client.data_sender_gstreamer_from_file import DataSenderGStreamerFromFile
from era_5g_netapp_client.client_gstreamer import NetAppClientGstreamer
from era_5g_netapp_client.client import FailedToConnect
import traceback

FROM_SOURCE = False  # Video from source


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

    # ip address or hostname of the server (middleware)
    server_ip = "localhost"
    user = "2c63955e-27cf-4b9a-980b-b70a54281d15"
    password = "test"
    task_id = "7d93728a-4a4c-4dae-8245-16b86f85b246"
    # to avoid exception in "except"
    client = None

    try:
        # creates the NetApp client with gstreamer extension
        client = NetAppClientGstreamer(server_ip, user, password, task_id, "True", get_results, True, True)
        # register the client with the NetApp
        client.register()
        if FROM_SOURCE:
            # creates a data sender which will pass images to the NetApp either from webcam ...
            data_src = f"v4l2src device=/dev/video0 ! video/x-raw, format=YUY2, width=640, height=480, " + \
                       "pixel-aspect-ratio=1/1 ! videoconvert ! appsink"
            DataSenderGStreamerFromSource(client.netapp_host, client.gstreamer_port, data_src, 15, 640, 480,
                                          False)
        else:
            # or from file
            DataSenderGStreamerFromFile(client.netapp_host, client.gstreamer_port, 15,
                                        "../../../../assets/2017_01_02_001021_s.mp4", 640, 480)
        # waits infinitely
        client.wait()
    except FailedToConnect as ex:
        print(f"Failed to connect to server ({ex})")
    except KeyboardInterrupt:
        print("Terminating...")
    except Exception as ex:
        traceback.print_exc()
        print(f"Failed to create client instance ({ex})")
    finally:
        if client is not None:
            client.disconnect()


if __name__ == '__main__':
    main()
