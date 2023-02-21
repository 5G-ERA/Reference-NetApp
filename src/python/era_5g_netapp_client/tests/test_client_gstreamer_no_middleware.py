import traceback
import signal
import os

from era_5g_netapp_client.data_sender_gstreamer_from_source import DataSenderGStreamerFromSource
from era_5g_netapp_client.data_sender_gstreamer_from_file import DataSenderGStreamerFromFile
from era_5g_netapp_client.client_gstreamer import NetAppClientGstreamer
from era_5g_netapp_client.client import FailedToConnect

# Video from source flag
FROM_SOURCE = False
# ip address or hostname of the computer, where the netapp is deployed
NETAPP_ADDRESS = os.getenv("NETAPP_ADDRESS", "127.0.0.1")
# port of the netapp's server
NETAPP_PORT = os.getenv("NETAPP_PORT", 5896)
# test video file
TEST_VIDEO_FILE = os.getenv("TEST_VIDEO_FILE", "../../../../assets/2017_01_02_001021_s.mp4")


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

    client = None
    sender = None

    def signal_handler(sig, frame):
        print(f"Terminating ({signal.Signals(sig).name})...")
        if sender is not None:
            sender.stop()
        if client is not None:
            client.disconnect()

    signal.signal(signal.SIGTERM, signal_handler)
    signal.signal(signal.SIGINT, signal_handler)

    try:
        # creates the NetApp client with gstreamer extension
        client = NetAppClientGstreamer(None, None, None, None, True, get_results, False, False, NETAPP_ADDRESS,
                                       NETAPP_PORT)
        # register the client with the NetApp
        client.register()
        if FROM_SOURCE:
            # creates a data sender which will pass images to the NetApp either from webcam ...
            data_src = f"v4l2src device=/dev/video0 ! video/x-raw, format=YUY2, width=640, height=480, " + \
                       "pixel-aspect-ratio=1/1 ! videoconvert ! appsink"
            sender = DataSenderGStreamerFromSource(client.netapp_host, client.gstreamer_port, data_src, 15, 640, 480,
                                                   False, daemon=True)
            sender.start()
        else:
            # or from file
            sender = DataSenderGStreamerFromFile(client.netapp_host, client.gstreamer_port, TEST_VIDEO_FILE, 15,
                                                 640, 480, daemon=True)
            sender.start()

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
