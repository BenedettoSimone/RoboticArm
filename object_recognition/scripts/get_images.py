#!/usr/bin/env python3.8
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import keras
import numpy as np
import os
from std_msgs.msg import String
import time
import keras.utils as image

# Initialize ROS node 'object_recognition'
rospy.init_node('object_recognition', anonymous=True)

# Initialize the CvBridge class to convert the images
bridge = CvBridge()

path = "catkin_ws/src/object_recognition/scripts/model/"

# Load CNN model
model = keras.models.load_model(path+"model.h5")
model.load_weights(path+"weights.h5")

with open(path+"labels.txt", 'r') as f:
    class_names = f.read().split('\n')


# send over the topic the classification result
def send_type(object_class):
    pub = rospy.Publisher('object_classification', String, queue_size=10)
    rospy.loginfo(object_class)
    pub.publish(object_class)



# Define a callback receiving the message
def image_callback(img_msg):
    
    # Print info about the image 
    rospy.loginfo(img_msg.header)

    # Convert the ROS Image message to a CV2 Image
    try:
        cv_image = bridge.imgmsg_to_cv2(img_msg, "bgr8")
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))
    
    
    cv2.imwrite("test.jpg", cv_image)
    img = image.load_img('test.jpg', target_size=(224,224))
    
    img = image.img_to_array(img)
    img = np.expand_dims(img, axis=0)
    images = np.vstack([img])
    prediction = model.predict(images)
    index = np.argmax(prediction)
    
    class_name = class_names[index]
    print(prediction)
    print(class_name)

    send_type(class_name)
    
    # Using cv2.putText(image, text, font, fontscale, color, thickness) method
    cv_image = cv2.putText(cv_image, class_name, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 
                   1, (255, 0, 0), 2, cv2.LINE_AA)
    
    # Show the converted image
    cv2.imshow("Camera Stream", cv_image)
    cv2.waitKey(3)
    
    time.sleep(1)


# Subscription to topic /seven_dof_arm/camera/image_raw/
sub_image = rospy.Subscriber("/seven_dof_arm/camera/image_raw/", Image, image_callback)


# Keep the program alive
while not rospy.is_shutdown():
    rospy.spin()
