#!/usr/bin/env python
# Description: simulate direct image transfer and reception - Subscriber

import rospy
import cv2 as cv
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

def resize(img, height=None, width=None):
    """resize the image

    Args:
        img (array): png image
        height (int, optional): pixel. Defaults to None.
        width ([int, optional): pixel. Defaults to None.

    Returns:
        image: the raw image
    """
    h, w = img.shape[0], img.shape[1]
    if height:
        width = int(w/h*height)
    else:
        height = int(h/w*width)
    # get the image
    target_img = cv.resize(img, dsize=(width, height))
    return target_img

def callback(data):
    """get the image data

    Args:
        data: message
    """
    img = CvBridge().imgmsg_to_cv2(data)
    # show
    cv.imshow("cute",resize(img, width=1000))
    # wait
    cv.waitKey(10)

if __name__ == "__main__":
    # node
    rospy.init_node("sub_image3")
    # topic
    sub = rospy.Subscriber("image_pool",data_class=Image,queue_size=10,callback=callback)
    # block
    rospy.spin()