#! /usr/bin/env python
# coding:utf-8
"""
client: 10 20
server: 10 + 20
"""

import rospy
from rospy_tutorials.srv import AddTwoInts, AddTwoIntsRequest, AddTwoIntsResponse

def callback(request):
    if not isinstance(request, AddTwoIntsRequest): return

    # 获取请求数据
    a = request.a
    b = request.b

    # 业务逻辑处理，返回响应结果
    c = a + b
    response = AddTwoIntsResponse()
    response.sum = c
    print(response.sum)
    return response


if __name__ == '__main__':
    # 创建节点
    rospy.init_node('server_node')

    # service 通讯的服务端 server
    # 服务端 server
    # 服务地址 /a/b/c/d
    service_name = '/shiyan/my_server'
    server = rospy.Service(service_name, AddTwoInts, callback)
    rospy.spin()