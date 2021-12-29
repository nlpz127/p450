// Reference: https://sir.upc.edu/projects/rostutorials/5-client-server_tutorial/index.html#a-server-program


#include <iomanip>
#include <iostream>
#include "ros/ros.h"
#include "simulate_control/c2s.h"
#include "simulate_control/t2t.h"

using namespace std;

simulate_control::c2s::Request all_req;
simulate_control::c2s::Response all_res;

bool callback(
    simulate_control::c2s::Request &req,
    simulate_control::c2s::Response& res){
    ROS_INFO("GET REQUEST: x=%.2f", (float)req.x);
    ROS_INFO("GET REQUEST: y=%.2f", (float)req.y);
    ROS_INFO("GET REQUEST: z=%.2f", (float)req.z);
    ROS_INFO("GET REQUEST: yaw=%.2f", (float)req.yaw);
    all_req.x = (float)req.x;
    all_req.y = (float)req.y;
    all_req.z = (float)req.z;
    all_req.yaw = (float)req.yaw;
    res.feedback = 1;
    return true;
}

int main(int argc, char **argv){

    // Some display settings
    cout.setf(ios::fixed);
    cout << setprecision(2);
    cout.setf(ios::left);
    cout.setf(ios::showpoint);

    // initial node
    ros::init(argc, argv, "service_topic");
    ros::NodeHandle nh;
    ros::ServiceServer simulate_control_server = nh.advertiseService("step1_service", &callback);
    ros::Publisher simulate_control_pub = nh.advertise<simulate_control::t2t>("step2_topic",100);

    

    ROS_INFO("READY");
    ros::Rate loop_rate(10);
    int number = 0;
    while(ros::ok()){
        ros::spinOnce();
        number += 1;
        simulate_control::t2t msg;
        msg.x = all_req.x;
        msg.y = all_req.y;
        msg.z = all_req.z;
        msg.yaw = all_req.yaw;

        simulate_control_pub.publish(msg);
        loop_rate.sleep();
    }

    return 0;
}