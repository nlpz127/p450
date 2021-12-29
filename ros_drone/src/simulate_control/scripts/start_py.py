#!/usr/bin/env python
import rospy
from simulate_control.srv import c2s, c2sRequest, c2sResponse

if __name__ == "__main__":
    rospy.init_node("now_client_py")
    service_name = "control_service"   # step1_service

    req = c2sRequest()
    print("INPUT X VALUE", "\t")
    req.x = float(input())
    print("INPUT Y VALUE", "\t")
    req.y = float(input())
    print("INPUT Z VALUE", "\t")
    req.z = float(input())
    print("INPUT YAW VALUE", "\t")
    req.yaw = float(input())

    try:
        # waiting...
        client = rospy.ServiceProxy(service_name, c2s)
        rospy.wait_for_service(service_name)
        
        # send request
        res = client.call(req)
        # log info
        rospy.loginfo("Send Request Successfully")
        rospy.loginfo(f"receive srv, srv.Response.result: {res.feedback}")
    except rospy.ServiceException as e:
        rospy.loginfo("Failed to call service")