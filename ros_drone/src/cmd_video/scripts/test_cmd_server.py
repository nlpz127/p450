#! /usr/bin/env python
# coding:utf-8
# Description: simulate the flight of drones - Server

import rospy
from cmd_video.srv import cmd, cmdRequest, cmdResponse

def if_success(request):
    """Judge if get the request

    Args:
        request: request message

    Returns:
        [int32]: Response status code
    """
    if not isinstance(request, cmdRequest):
        # if error
        print("Error request")
        return
    else:
        # get the request
        print(f"Get commond x={request.x}, y={request.y}, z={request.z}, yaw={request.yaw}")
        flag = 0
    return cmdResponse(flag)

def cmd_server():
    """Server receives information
    """
    # node
    rospy.init_node('server_node')
    # service
    service_name = '/my/serHostFBLR'
    # connection
    server = rospy.Service(service_name, cmd, if_success)
    print("Ready to get the fly cmd...")
    # Hover, continue to accept instructions
    rospy.spin()

if __name__ == '__main__':
    # demo
    cmd_server()