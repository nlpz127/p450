#!/usr/bin/env python
# Description: simulate the transmission of a two-dimensional array - Subscriber

import rospy
import os
from std_msgs.msg import String, Int32MultiArray

def save_video_array(data):
    """save array data to txt

    Args:
        data (string): array
    """
    with open("./pic_array/save.txt","a+")as file:
        file.write("I heard %s "%str(data.data)+"\n")

def callback(data):
    """Receive feedback

    Args:
        data: message
    """
    # save data
    save_video_array(data)
    rospy.loginfo(f"I heard {data.data}")

def listener():
    # node
    rospy.init_node('video_listener', anonymous=True) # unique
    # topic
    rospy.Subscriber('vv', Int32MultiArray, callback)
    # block
    rospy.spin()


if __name__ == "__main__":
    #print(os.getcwd())
    # Subscribe messages
    listener()