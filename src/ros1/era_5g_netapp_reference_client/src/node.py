#!/usr/bin/env python3
# license removed for brevity
import rospy
import cv2
#from era_5g_netapp_client.client_gstreamer import NetAppClientGstreamer as NetAppClient
from era_5g_netapp_client.client_gstreamer import NetAppClientGstreamer
from era_5g_netapp_client.data_sender_gstreamer import DataSenderGStreamer
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# the IP address or hostname of the NetApp interface
server_ip = "IP_OR_HOSTNAME"  

bridge = CvBridge()

def results_callback(data):
    print(data)
    #results_pub.publish(json.dumps(data))

def image_callback(image):
    # convert recieved image to opencv
    cv_image = bridge.imgmsg_to_cv2(image, desired_encoding='passthrough')
    # color correction
    im_bgr = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)  
    # either send the data using the gstreamer data sender
    #data_sender.send_image(im_bgr)

    # or use the HTTP transort to the /image endpoint
    client.send_image(im_bgr, image.header.stamp)
    

if __name__ == '__main__':
    try:
        rospy.init_node("netapp_client")
        # create the client (or use NetAppClient if you dont want to use gstreamer)
        client = NetAppClientGstreamer(server_ip, 5896, results_callback)
        client.register()
        # data sender is only needed when gstreamer is used
        # data_sender = DataSenderGStreamer(server_ip, client.gstreamer_port, 30)
        results_pub = rospy.Publisher("results", String, queue_size=10)
        sub = rospy.Subscriber("/usb_cam/image_raw", Image, image_callback)
        
        

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
