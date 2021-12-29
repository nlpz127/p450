#!/usr/bin/env python
# Description: simulate direct image transfer and reception - Publisher

import rospy
import cv2 as cv
from cv_bridge.core import CvBridge
from sensor_msgs.msg import Image
import os

if __name__ == "__main__":
    # node
    rospy.init_node("pub_image3")
    # print(os.getcwd())
    
    # Read local image information
    img_path =  "./pic/cute.png"
    # print(img_path)
    img = cv.imread(img_path)
    print(type(img))
    
    # publish config
    pub = rospy.Publisher("image_pool", data_class=Image, queue_size=10)
    loop_rate = rospy.Rate(2)

    # # The current publisher has not stopped
    while not rospy.is_shutdown():
        
        # get pic message
        msg = CvBridge().cv2_to_imgmsg(img)
        # publish
        pub.publish(msg)
        # log
        print("I have published an image, [{}x{}]".format(msg.height, msg.width))
        # frequency
        loop_rate.sleep()