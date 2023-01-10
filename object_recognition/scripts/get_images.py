#!/usr/bin/env python3.8
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import keras
import numpy as np
import os

# Initialize ROS node 'object_recognition'
rospy.init_node('object_recognition', anonymous=True)

# Initialize the CvBridge class to convert the images
bridge = CvBridge()

path = "catkin_ws/src/object_recognition/scripts/"

# Load CNN model
model = keras.models.load_model(path+"model.h5")
model.load_weights(path+"weights.h5")

with open(path+"labels.txt", 'r') as f:
    class_names = f.read().split('\n')


# Define a callback receiving the message
def image_callback(img_msg):
    
    # Print info about the image 
    rospy.loginfo(img_msg.header)

    # Convert the ROS Image message to a CV2 Image
    try:
        cv_image = bridge.imgmsg_to_cv2(img_msg, "bgr8")
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))
    
    img = cv2.resize(cv_image, (400,400), interpolation = cv2.INTER_AREA)
    img = np.expand_dims(img, axis=0)

    # run the inference
    prediction = model.predict(img)
    index = np.argmax(prediction)

    class_name = class_names[index]
    
    print(prediction)
    print(class_name)

    # Show the converted image
    cv2.imshow("Camera Stream", cv_image)
    cv2.waitKey(3)


# Subscription to topic /seven_dof_arm/camera/image_raw/
sub_image = rospy.Subscriber("/seven_dof_arm/camera/image_raw/", Image, image_callback)


# Keep the program alive
while not rospy.is_shutdown():
    rospy.spin()
