#!/usr/bin/env python3
# license removed for brevity
import json
import rospy
import cv2
from era_5g_client.client_base import NetAppClientBase
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os



try:
    NETAPP_ADDRESS = os.environ["NETAPP_ADDRESS"]
except KeyError as e:
    raise Exception(f"Failed to run node, env variable {e} not set.")

try:
    NETAPP_PORT = os.environ["NETAPP_PORT"]
except KeyError as e:
    raise Exception(f"Failed to run node, env variable {e} not set.")

try:
    INPUT_TOPIC = os.environ["INPUT_TOPIC"]
except KeyError as e:
    raise Exception(f"Failed to run node, env variable {e} not set.")

try:
    OUTPUT_TOPIC = os.environ["OUTPUT_TOPIC"]
except KeyError as e:
    raise Exception(f"Failed to run node, env variable {e} not set.")

bridge = CvBridge()

image_buffer = dict()

results_pub = None

tmstp = 1

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
        
        im_bgr = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)  
        results_pub.publish(bridge.cv2_to_imgmsg(im_bgr, encoding='rgb8'))
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
    global tmstp
    # or use the HTTP transort to the /image endpoint
    client.send_image_http(im_bgr, tmstp)
    image_buffer[tmstp] = im_bgr
    tmstp += 1
    

if __name__ == '__main__':
    try:
        rospy.init_node("netapp_client")
        # create an instance of NetApp client with results callback
        client = NetAppClientBase(results_callback)
        client.register(NETAPP_ADDRESS, NETAPP_PORT, ws_data=True)
        
        # data sender is only needed when gstreamer is used
        # data_sender = DataSenderGStreamer(server_ip, client.gstreamer_port, 30)

        # create a publisher for the images with embedded bounding boxes
        results_pub = rospy.Publisher(OUTPUT_TOPIC, Image, queue_size=10)
        sub = rospy.Subscriber(INPUT_TOPIC, Image, image_callback)
        
        

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
