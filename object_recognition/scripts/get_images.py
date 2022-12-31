#!/usr/bin/env python3.8
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError


# Initialize ROS node 'object_recognition'
rospy.init_node('object_recognition', anonymous=True)

# Initialize the CvBridge class to convert the images
bridge = CvBridge()


# Define a callback receiving the message
def image_callback(img_msg):
    
    # Print info about the image 
    rospy.loginfo(img_msg.header)

    # Convert the ROS Image message to a CV2 Image
    try:
        cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))

    # Show the converted image
    cv2.imshow("Camera Stream", cv_image)
    cv2.waitKey(3)


# Subscription to topic /seven_dof_arm/camera/image_raw/
sub_image = rospy.Subscriber("/seven_dof_arm/camera/image_raw/", Image, image_callback)


# Keep the program alive
while not rospy.is_shutdown():
    rospy.spin()