import argparse
from argparse import ArgumentTypeError
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from std_msgs.msg import Header
from sensor_msgs.msg import Image

from era_5g_service_interfaces.srv import Start
from era_5g_service_interfaces.srv import Stop

# Default 5G-ERA ML Control Services base name.
DEFAULT_ML_SERVICE_BASE_NAME = 'ml_control_services'


class MLControlServices(Node):
    """
    MLControlServices class, which is a subclass of the Node class.
    The node provides 5G-ERA ML Control Services.
    """

    def __init__(self, service_base_name: str):
        """
        Class constructor to set up the node.
        """
        super().__init__('ml_control_services')

        # Create 5G-ERA ML Control Services.
        self.start_service = self.create_service(Start, service_base_name + '_start', self.start_request_callback)
        self.stop_service = self.create_service(Stop, service_base_name + '_stop', self.stop_request_callback)

        self.nodes = {}

    def stop_request_callback(self, request: Stop.Request, response: Stop.Response):
        """
        Callback function.
        This function gets called on Stop request.

        :param request: Stop request
        :param response: Stop response
        :return: Stop response
        """
        response.success = True
        response.message = 'Topics successfully destroyed'
        self.get_logger().info('Incoming "stop" request: \n'
                               'task_id: "%s"\n'
                               'response: \nsuccess: %r, message: "%s"' %
                               (request.task_id,
                                response.success, response.message))
        # Destroy dynamically created node.
        self.executor.remove_node(self.nodes[request.task_id])
        self.nodes[request.task_id].destroy_node()
        del self.nodes[request.task_id]

        # Return generated topic names.
        return response

    def start_request_callback(self, request: Start.Request, response: Start.Response):
        """
        Callback function.
        This function gets called on Start request.

        :param request: Start request
        :param response: Start response
        :return: Start response
        """
        response.success = True
        response.message = 'Topics successfully generated'
        # Generate task id.
        response.task_id = str(round(time.time() * 1000))
        # Generate data topic name for the robot.
        response.data_topic = 'data_topic_' + response.task_id
        # Generate result topic name for the robot.
        response.result_topic = 'result_topic_' + response.task_id
        # Display the request and the response on the console.
        self.get_logger().info('Incoming "start" request: \n'
                               'robot_worker_setting: "%s"\n'
                               'response: \nsuccess: %r, message: "%s", '
                               'task_id: "%s", data_topic: "%s", result_topic: "%s", worker_constraints: "%s"' %
                               (request.robot_worker_setting,
                                response.success, response.message,
                                response.task_id, response.data_topic, response.result_topic,
                                response.worker_constraints))

        # Dynamically create node for the generated topics.
        self.nodes[response.task_id] = rclpy.create_node("node_" + str(response.task_id))

        # Create publisher for answering the robot, therefore the subscriber_topic_name is set.
        publisher = self.nodes[response.task_id].create_publisher(String, response.result_topic, qos_profile_sensor_data)

        def result_callback(image: Image):
            """
            Callback function.
            This function is called when the image from the robot is received.
            """
            msg = String()
            msg.data = 'The answer to the image: %s' % image.header.frame_id
            self.nodes[response.task_id].get_logger().info('Publishing: %s' % msg.data)
            publisher.publish(msg)

        # Create a subscription for receiving images from the robot, therefore the publisher_topic_name is set.
        subscription = self.nodes[response.task_id].create_subscription(Image, response.data_topic, result_callback,
                                                                        qos_profile_sensor_data)

        # Add node to running executor.
        self.executor.add_node(self.nodes[response.task_id])

        # Return generated topic names.
        return response


def main():
    """Main function."""
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter,
                                     description='ROS2 5G-ERA ML Control Service app')
    parser.add_argument('--ml_service_base_name', '-n', dest='ml_service_base_name', action='store',
                        help='5G-ERA ML Control Service base name', default=DEFAULT_ML_SERVICE_BASE_NAME)
    arguments = parser.parse_args()

    # Initialize the rclpy library.
    rclpy.init()

    # Create the Service node.
    service = MLControlServices(arguments.ml_service_base_name)

    try:
        rclpy.spin(service)
    except KeyboardInterrupt:
        pass
    except BaseException:
        print('Exception in 5G-ERA ML Control Services:', file=sys.stderr)
        raise
    finally:
        service.destroy_node()
        rclpy.shutdown()
        pass


if __name__ == '__main__':
    main()
