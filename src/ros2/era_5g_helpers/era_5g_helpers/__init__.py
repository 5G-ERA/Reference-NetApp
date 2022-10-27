import os
import requests
from requests.adapters import HTTPAdapter, Retry
from requests.exceptions import Timeout
from requests.exceptions import ConnectionError

DEBUG = True

NETAPP_ID = "00000000-0000-0000-0000-000000000000"
if os.getenv("NETAPP_ID"):
    NETAPP_ID = os.getenv("NETAPP_ID")
NETAPP_ID_ROS = NETAPP_ID.replace("-", "_")
MIDDLEWARE_ADDRESS = "http://localhost"
if os.getenv("MIDDLEWARE_ADDRESS"):
    MIDDLEWARE_ADDRESS = os.getenv("MIDDLEWARE_ADDRESS")
MIDDLEWARE_REPORT_INTERVAL = 1
if os.getenv("MIDDLEWARE_REPORT_INTERVAL"):
    MIDDLEWARE_REPORT_INTERVAL = float(os.getenv("MIDDLEWARE_REPORT_INTERVAL"))

retries = Retry(
    total=0,
    read=0,
    connect=0,
    backoff_factor=0,
    status_forcelist=[429, 500, 502, 503, 504]
)

session = requests.Session()
adapter = HTTPAdapter(max_retries=retries)
session.mount(MIDDLEWARE_ADDRESS, adapter)
connection_error = False


def send_middleware_heart_beat_request(node, headers, json):
    global connection_error
    if not connection_error:
        node.get_logger().info(f"{node.get_name()} is sending heart_beat to middleware: {json}")
        try:
            response = session.post(MIDDLEWARE_ADDRESS, headers=headers, json=json, timeout=(0.2, 0.2))
            node.get_logger().info(f"{node.get_name()} middleware heart_beat response: {response.content}")
        except Exception as e:
            node.get_logger().info(f"{node.get_name()} failed to connect to the middleware address: {MIDDLEWARE_ADDRESS}")
            print(e)
            connection_error = True


def get_path_to_assets() -> str:
    """
    Gets path to assets folder from environment variable.
    """

    path_to_assets = os.getenv("ROS2_5G_ERA_ASSETS_PATH")

    if not path_to_assets:
        if DEBUG:
            print("Environment variable ROS2_5G_ERA_ASSETS_PATH not set.")
            path_to_assets = os.path.abspath("assets")
            print(f"Trying to set path to: {path_to_assets}")
        else:
            raise EnvironmentError("Path to assets not set!")

    if not os.path.exists(path_to_assets):
        raise EnvironmentError("Path to assets does not exist!")

    return path_to_assets


def get_path_from_env(env_var_name, default_dir="assets") -> str:
    """
    Gets directory path from the given environment variable.
    (This is a more general version of the previous function.)
    """

    dir_path = os.getenv(env_var_name)

    if not dir_path:
        if DEBUG:
            print(f"Environment variable {env_var_name} not set.")
            dir_path = os.path.abspath(default_dir)
            print(f"Trying to set path to: {dir_path}")
        else:
            raise EnvironmentError(f"Environment variable {env_var_name} not set!")

    if not os.path.exists(dir_path):
        raise EnvironmentError(f"Path {dir_path} does not exist!")

    return dir_path


