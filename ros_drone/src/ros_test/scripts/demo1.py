#!/usr/bin/env python

import rospy

if __name__ == '__main__':
    node_name = "ros_demo1"
    rospy.init_node(node_name)
    print("my first ros demo test")
    rospy.spin()
