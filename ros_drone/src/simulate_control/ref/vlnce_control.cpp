/***************************************************************************************************************************
* vlnce_control.cpp
*
* Update Time: 2021.12.19
*
* Introduction:  test function for sending ControlCommand.msg from vlnce_terminal
***************************************************************************************************************************/

#include <iostream>
#include <iomanip>

#include <ros/ros.h>

#include <prometheus_msgs/ControlCommand.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Path.h>

#include "KeyboardEvent.h"

#define VEL_XY_STEP_SIZE 0.1
#define VEL_Z_STEP_SIZE 0.1
#define YAW_STEP_SIZE 0.08
#define TRA_WINDOW 2000
#define NODE_NAME "vlnce_control"

using namespace std;

// Command message set
prometheus_msgs::ControlCommand Command_to_pub;

// Publisher 
ros::Publisher move_pub_vlnce;

// Precision
const float EPSINON = 0.00001; 

void mainloop1();
void generate_com(int Move_mode, float state_desired[4]);

int main(int argc, char **argv){

    ros::init(argc, argv, "vlnce_control");
    ros::NodeHandle nh;

    // publish controling command 
    move_pub_vlnce = nh.advertise<prometheus_msgs::ControlCommand>("/prometheus/control_command", 10);

    // initial, waiting control message
    Command_to_pub.Mode                                = prometheus_msgs::ControlCommand::Idle;
    Command_to_pub.Command_ID                          = 0;
    Command_to_pub.source = NODE_NAME;
    Command_to_pub.Reference_State.Move_mode           = prometheus_msgs::PositionReference::XYZ_POS;       // default 0, normal 
    Command_to_pub.Reference_State.Move_frame          = prometheus_msgs::PositionReference::ENU_FRAME;     // default 0, ENU_FRAME, for take off
    Command_to_pub.Reference_State.position_ref[0]     = 0;
    Command_to_pub.Reference_State.position_ref[1]     = 0;
    Command_to_pub.Reference_State.position_ref[2]     = 0;
    Command_to_pub.Reference_State.velocity_ref[0]     = 0;
    Command_to_pub.Reference_State.velocity_ref[1]     = 0;
    Command_to_pub.Reference_State.velocity_ref[2]     = 0;
    Command_to_pub.Reference_State.acceleration_ref[0] = 0;
    Command_to_pub.Reference_State.acceleration_ref[1] = 0;
    Command_to_pub.Reference_State.acceleration_ref[2] = 0;
    Command_to_pub.Reference_State.yaw_ref             = 0;

    // Some display settings
    cout.setf(ios::fixed);
    cout << setprecision(2);
    cout.setf(ios::left);
    cout.setf(ios::showpoint);


    // Default terminal input control
    int Remote_Mode;
    cout << ">>>>>>>>>>>>>>>> Welcome to use VLNCE Terminal Control <<<<<<<<<<<<<<<<"<< endl;
    cout << "Default Input Control..." << endl;
    Remote_Mode = 0;

    mainloop1();

    return 0;
}

void mainloop1()
{
    int Control_Mode = 0;
    int Move_mode = 0;
    int Move_frame = 0;
    float state_desired[4];
    int confirm_unlock = 0;

    while(ros::ok()){

        if(confirm_unlock == 0){

            // takeoff - x=0, y=0, z=0.5, yaw=0
            // cout << "Default 1 for Takeoff" <<endl;
            cout << "pls input 1 to make the drone take off." << endl;
            cin >> Control_Mode;
            
            if(Control_Mode == prometheus_msgs::ControlCommand::Takeoff){
                Command_to_pub.header.stamp = ros::Time::now();
                Command_to_pub.Mode = prometheus_msgs::ControlCommand::Takeoff;
                Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
                Command_to_pub.source = NODE_NAME;
                move_pub_vlnce.publish(Command_to_pub);
            }

            // unlock
            while(confirm_unlock == 0){
                cout << "Pls confirm that you have unlocked the drone and the drone has taken off. (0 for No, 1 for Yes)" << endl;
                cin >> confirm_unlock; 
            }
        }
        else{
            cout << "Congraduations! Your order has taken effect." << endl;
            // default commands
            Control_Mode = 4;   // For move
            Move_mode = 0;      // Command.Reference_State.Move_mode: 0 for XYZ_POS
            Move_frame  = 1;    // Command.Reference_State.Move_frame: 1 for BODY_FRAME

            if(Control_Mode == prometheus_msgs::ControlCommand::Move){
                // for move
                cout << "Please input the reference state [x y z yaw]: " << endl;
                cout << "setpoint_t[0] --- x [m] : "<< endl;
                cin >> state_desired[0];
                cout << "setpoint_t[1] --- y [m] : "<< endl;
                cin >> state_desired[1];
                cout << "setpoint_t[2] --- z [m] : "<< endl;
                cin >> state_desired[2];
                cout << "setpoint_t[3] --- yaw [du] : "<< endl;
                cout << "[Notes]: + Counterclockwise, - Clockwise" << endl;
                cin >> state_desired[3];
                cout << "The order has been sent." << endl;

                if(abs(state_desired[0]+1.5) < EPSINON || abs(state_desired[0]+1.5) == EPSINON){ 
                    cout << "The drone begins to landing..." << endl;       // if x == -1.5
                    Control_Mode = 3; // land
                    Command_to_pub.header.stamp = ros::Time::now();
                    Command_to_pub.Mode = prometheus_msgs::ControlCommand::Land;
                    Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
                    Command_to_pub.source = NODE_NAME;
                    move_pub_vlnce.publish(Command_to_pub);
                    break;
                }
                
                Command_to_pub.header.stamp = ros::Time::now();
                Command_to_pub.Mode = prometheus_msgs::ControlCommand::Move;
                Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
                Command_to_pub.source = NODE_NAME;
                Command_to_pub.Reference_State.Move_mode  = Move_mode;          // 0 for XYZ_POS
                Command_to_pub.Reference_State.Move_frame = Move_frame;         // 1 for BODY_FRAME
                Command_to_pub.Reference_State.time_from_start = -1;
                generate_com(Move_mode, state_desired);
                move_pub_vlnce.publish(Command_to_pub);
            }
        }    
        cout << "....................................................." <<endl;
        sleep(1.0);
} 
void generate_com(int Move_mode, float state_desired[4]){

    //# Move_mode 2-bit value:
    //# 0 for position, 1 for vel, 1st for xy, 2nd for z.
    //#                   xy position     xy velocity
    //# z position       	0b00(0)       0b10(2)
    //# z velocity		0b01(1)       0b11(3)

    if((Move_mode & 0b10) == 0){ //xy channel
        Command_to_pub.Reference_State.position_ref[0] = state_desired[0];
        Command_to_pub.Reference_State.position_ref[1] = state_desired[1];
        Command_to_pub.Reference_State.velocity_ref[0] = 0;
        Command_to_pub.Reference_State.velocity_ref[1] = 0;
    }
    else{
        Command_to_pub.Reference_State.position_ref[0] = 0;
        Command_to_pub.Reference_State.position_ref[1] = 0;
        Command_to_pub.Reference_State.velocity_ref[0] = state_desired[0];
        Command_to_pub.Reference_State.velocity_ref[1] = state_desired[1];
    }

    if((Move_mode & 0b01) == 0){ //z channel
        Command_to_pub.Reference_State.position_ref[2] = state_desired[2];
        Command_to_pub.Reference_State.velocity_ref[2] = 0;
    }
    else{
        Command_to_pub.Reference_State.position_ref[2] = 0;
        Command_to_pub.Reference_State.velocity_ref[2] = state_desired[2];
    }

    Command_to_pub.Reference_State.acceleration_ref[0] = 0;
    Command_to_pub.Reference_State.acceleration_ref[1] = 0;
    Command_to_pub.Reference_State.acceleration_ref[2] = 0;

    Command_to_pub.Reference_State.yaw_ref = state_desired[3]/180.0*M_PI;

}