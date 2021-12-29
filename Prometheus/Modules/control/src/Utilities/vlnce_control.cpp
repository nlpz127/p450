/***************************************************************************************************************************
* vlnce_control.cpp
*
* Update Time: 2021.12.19
*
* Introduction:  test function for sending ControlCommand.msg from vlnce_terminal
***************************************************************************************************************************/
#include <ros/ros.h>
#include <iostream>


#include <prometheus_msgs/ControlCommand.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Path.h>

// added Header File
#include <prometheus_msgs/c2s.h>
// added Header File

#define VEL_XY_STEP_SIZE 0.1
#define VEL_Z_STEP_SIZE 0.1
#define YAW_STEP_SIZE 0.08
#define TRA_WINDOW 2000
#define NODE_NAME "vlnce_control"
#define EPS 0.00001

using namespace std;

//即将发布的command
prometheus_msgs::ControlCommand Command_to_pub;
//发布
ros::Publisher move_pub_vlnce;
//即将接收的command
prometheus_msgs::c2s::Request all_req;
prometheus_msgs::c2s::Response all_res;

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>　函数声明　<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void mainloop1();
void generate_com(int Move_mode, float state_desired[4]);
bool callback(prometheus_msgs::c2s::Request &req, prometheus_msgs::c2s::Response& res);

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>　主函数　<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "vlnce_control");
    ros::NodeHandle nh;

    // 【接收】　控制指令
    ros::ServiceServer control_server = nh.advertiseService("control_service", &callback);
    //　【发布】　控制指令
    move_pub_vlnce = nh.advertise<prometheus_msgs::ControlCommand>("/prometheus/control_command", 10);

    // 初始化命令 - Idle模式 电机怠速旋转 等待来自上层的控制指令
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

    //固定的浮点显示
    cout.setf(ios::fixed);
    //setprecision(n) 设显示小数精度为n位
    cout << setprecision(2);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    //cout.setf(ios::showpos);

    // 默认终端输入控制
    int Remote_Mode;
    cout << ">>>>>>>>>>>>>>>> Welcome to use VLNCE Terminal Control <<<<<<<<<<<<<<<<"<< endl;
    cout << "Default Input Control..."<<endl;
    Remote_Mode = 0;

    cout << "Now pls input number to control..."<<endl;
    mainloop1();

    return 0;
}



bool callback(
    prometheus_msgs::c2s::Request &req,
    prometheus_msgs::c2s::Response& res){
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
            cout << "Input 1 for Takeoff" <<endl;
            cin >> Control_Mode;
            


            if(Control_Mode == prometheus_msgs::ControlCommand::Takeoff){
                Command_to_pub.header.stamp = ros::Time::now();
                Command_to_pub.Mode = prometheus_msgs::ControlCommand::Takeoff;
                Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
                Command_to_pub.source = NODE_NAME;
                move_pub_vlnce.publish(Command_to_pub);
            }
            // unlock
            cout << "Pls confirm that if you have unlocked the drone. (0 for No, 1 for Yes)" << endl;
            cin >> confirm_unlock; 
        }
        else{
            cout << "Congraduations! The drone has taken off." << endl;
            // default commands
            Control_Mode = 4;   // For move
            Move_mode = 0;      // Command.Reference_State.Move_mode: 0 for XYZ_POS
            Move_frame  = 1;    // Command.Reference_State.Move_frame: 1 for BODY_FRAME

            if(Control_Mode == prometheus_msgs::ControlCommand::Move){
                // for move
                ros::spinOnce();
                cout << "Waiting for the reference state [x y z yaw]: "<< endl;
                state_desired[0] = all_req.x;
                cout << "Get setpoint_t[0] --- x [m] : "<< state_desired[0] << endl;
                
                state_desired[1] = all_req.y;
                cout << "Get setpoint_t[1] --- y [m] : "<< state_desired[1] << endl;

                state_desired[2] = all_req.z;
                cout << "Get setpoint_t[2] --- z [m] : "<< state_desired[2] << endl;

                state_desired[3] = all_req.yaw;
                cout << "Get setpoint_t[3] --- yaw [du] (+nishizhen): "<< state_desired[3] << endl;

                if(abs(state_desired[0]+1.5) < EPS || abs(state_desired[0]+1.5) == EPS){        // if x == -1.5

                    cout << "landing" << endl;
                    Control_Mode = 3; // land
                    Command_to_pub.header.stamp = ros::Time::now();
                    Command_to_pub.Mode = prometheus_msgs::ControlCommand::Land;
                    Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
                    Command_to_pub.source = NODE_NAME;
                    move_pub_vlnce.publish(Command_to_pub);
                    confirm_unlock = 0;
                    goto LAND;
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
                all_req.x = 0;
                all_req.y = 0;
                all_req.z = 0;
                all_req.yaw = 0;



            }
        }
		LAND:
        cout << "....................................................." <<endl;
        sleep(1.0);
    } 

}

void generate_com(int Move_mode, float state_desired[4])
{
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
