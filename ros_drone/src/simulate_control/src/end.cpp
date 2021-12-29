#include <iomanip>
#include <iostream>
#include "ros/ros.h"
#include "simulate_control/t2t.h"

using namespace std;

void msgCallback(const simulate_control::t2t::ConstPtr &msg){
    ROS_INFO("------------------------------");
    ROS_INFO("receive x msg = %.2f", msg->x);
    ROS_INFO("receive y msg = %.2f", msg->y);
    ROS_INFO("receive z msg = %.2f", msg->z);
    ROS_INFO("receive yaw msg = %.2f", msg->yaw);
}

int main(int argc, char **argv){

    // Some display settings
    cout.setf(ios::fixed);
    cout << setprecision(2);
    cout.setf(ios::left);
    cout.setf(ios::showpoint);

    ros::init(argc, argv, "now_subscriber");
    ros::NodeHandle nh;
    ros::Subscriber simulate_control = nh.subscribe("step2_topic", 100, msgCallback);

    ros::spin();

    return 0;
}