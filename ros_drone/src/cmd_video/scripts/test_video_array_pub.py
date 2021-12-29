#!/usr/bin/env python
# Description: simulate the transmission of a two-dimensional array - Publisher

import rospy
import numpy as np
from std_msgs.msg import String, Int32MultiArray

def publisher():
    """Send a message at a frequency of 1 time per second.
    """
    # init node and topic
    rospy.init_node('video_publisher', anonymous=True)
    pub = rospy.Publisher('vv', Int32MultiArray, queue_size=10)
    
    # rate - frequency
    rate = rospy.Rate(1) # 1hz = 1 times/s 
    
    # The current publisher has not stopped
    while not rospy.is_shutdown():
        # array
        # np.random.seed(1207)
        
        # create instance
        p_list = Int32MultiArray()
        
        # Three-dimensional array
        array_ = np.random.randint(1,10,3) 
        vec = array_.flatten() # flatten, ignore, designed for higher dimensions - publisher
        # to list 
        p_list.data = vec.tolist() 
        
        # print log information
        rospy.loginfo(p_list.data)
        # publish
        pub.publish(p_list)
        # frequency
        rate.sleep()

if __name__ == '__main__':
    try:
        # publish messages
        publisher()
    except rospy.ROSInterruptException:
        pass
