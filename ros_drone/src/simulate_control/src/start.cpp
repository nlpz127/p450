#include <iomanip>
#include <iostream>
#include <cstdlib>
#include "ros/ros.h"
#include "simulate_control/c2s.h"


using namespace std;

int main(int argc, char **argv){

    ros::init(argc, argv, "now_client");
    ros::NodeHandle nh;
    ros::ServiceClient simulate_control_client = nh.serviceClient<simulate_control::c2s>("step1_service");

    // Some display settings
    // cout.setf(ios::fixed);
    // cout << setprecision(2);
    // cout.setf(ios::left);
    // cout.setf(ios::showpoint);

    while(1){
        simulate_control::c2s commands;
        cout << "Please input the reference state [x y z yaw]: " << std::endl;
        cout << "INPUT --- x [m] : "<< endl;
        cin >> commands.request.x;
        cout << "INPUT --- y [m] : "<< endl;
        cin >> commands.request.y;
        cout << "INPUT --- z [m] : "<< endl;
        cin >> commands.request.z;
        cout << "INPUT --- yaw [du] [Notes]: + Counterclockwise, - Clockwise: "<< endl;
        cin >> commands.request.yaw;

        if(simulate_control_client.call(commands)){
            ROS_INFO("THE CLIENT HAS SENT THE COMMANDS.");
            // cout << commands.response.feedback << endl;
        }
        else{
            ROS_ERROR("FAILED TO SEND COMMANDS!");
            return 1;
        }
    }
    return 0;
}