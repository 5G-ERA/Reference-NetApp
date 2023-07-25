#!/usr/bin/env python3
# license removed for brevity
import json

from era_5g_client.dataclasses import NetAppLocation
import rospy
import cv2
from era_5g_client.client_base import NetAppClientBase
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os

# ip address or hostname of the netapp
NETAPP_ADDRESS = os.getenv("NETAPP_ADDRESS", "127.0.0.1")
# netapp port
NETAPP_PORT = os.getenv("NETAPP_PORT", 5896)

bridge = CvBridge()

image_buffer = dict()

results_pub = None

def results_callback(data):
    global image_buffer
    if type(data) != dict:
        return
    timestamp = data.get("timestamp", None)
    if timestamp is None:
        return
    else:
        timestamp = int(timestamp)
    try:
        frame = image_buffer.pop(timestamp)
        if frame is None:
            return
        detections = data["detections"]
        for d in detections:
            score = float(d["score"])
            cls_name = d["class_name"]
            # Draw detection into frame.
            x1, y1, x2, y2 = [int(coord) for coord in d["bbox"]]
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 1)
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(
                frame,
                f"{cls_name} ({score * 100:.0f})%",
                (x1, y1 - 5),
                font,
                0.5,
                (0, 255, 0),
                1,
                cv2.LINE_AA,
            )
        
        im_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)  
        results_pub.publish(bridge.cv2_to_imgmsg(im_rgb, encoding='rgb8'))
        # remove all images with older timestamps than last received result 
        image_buffer = {k:v for k, v in image_buffer.items() if k > timestamp}
        

    except KeyError as ex:
        print(f"Frame with timestamp {ex} not found")
    #results_pub.publish(json.dumps(data))

def image_callback(image):
    # convert recieved image to opencv
    cv_image = bridge.imgmsg_to_cv2(image, desired_encoding='passthrough')
    # color correction
    im_bgr = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)  

    # or use the HTTP transort to the /image endpoint
    client.send_image_http(im_bgr, image.header.stamp.to_nsec())
    image_buffer[image.header.stamp.to_nsec()] = im_bgr
    

if __name__ == '__main__':
    try:
        rospy.init_node("netapp_client")
        # create an instance of NetApp client with results callback
        client = NetAppClientBase(results_callback)
        # register with an ad-hoc deployed NetApp
        client.register(NetAppLocation(NETAPP_ADDRESS, NETAPP_PORT))

        # create a publisher for the images with embedded bounding boxes
        results_pub = rospy.Publisher("results", Image, queue_size=10)
        sub = rospy.Subscriber("/input_image", Image, image_callback)
        
        

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
