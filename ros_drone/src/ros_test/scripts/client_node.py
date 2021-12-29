#!/usr/bin/env python
# coding:utf-8
import rospy
from rospy_tutorials.srv import AddTwoInts, AddTwoIntsRequest, AddTwoIntsResponse

if __name__ == '__main__':
    # 创建节点
    rospy.init_node('client_node')

    # 服务名称地址
    service_name = '/shiyan/my_server'
    client = rospy.ServiceProxy(service_name, AddTwoInts)

    # 确保 server 是存在的，等待服务开启, 阻塞代码
    rospy.wait_for_service(service_name)

    # 给 server 发请求
    request = AddTwoIntsRequest()
    request.a = 16
    request.b = 8
    response = client.call(request)

    if isinstance(response, AddTwoIntsResponse):
        print(response.sum)

    rospy.spin()