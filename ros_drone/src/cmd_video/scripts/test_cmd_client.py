#!/usr/bin/env python
# coding:utf-8
# Description: simulate the flight of drones - Client

import rospy
from cmd_video.srv import cmd, cmdRequest, cmdResponse

def cmd_client(x, y, z, yaw):
    """Client sends instructions

    Args:
        x (float32): x m
        y (float32): y m
        z (float32): z m
        yaw (float32): yaw angle value
    """
    # node
    rospy.init_node('client_node')
    # waiting for the service to start, blocking state
    rospy.wait_for_service('/my/serHostFBLR')
    try:
        # Service agent
        client = rospy.ServiceProxy('/my/serHostFBLR', cmd)

        # send request to server
        ## create instance
        request = cmdRequest()
        request.x = x
        request.y = y
        request.z = z
        request.yaw = yaw
        # send request
        response = client.call(request)

        if isinstance(response, cmdResponse):
            print("Commond type correct and send out successfully!")

    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")


if __name__== "__main__":
    # Continuous input, space separation, press enter to stop
    xx, yy, zz, yawy = map(int,input().split(' '))
    # Send instructions
    cmd_client(xx, yy, zz, yawy)
