/*
 *功能：接收跟踪程序发布的目标位置，转换成控制机器人移动的指令
 *Author:Andy Wei
 *Date:2018年7月4日
 */
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <cmath>
#include "pid.h"

int image_width=640;
int image_height=480;
float image_turn;//读取目标位置在图像中的坐标值
float image_depth;


ros::Publisher pub;//发布命令
float speed_x = 0;
float turn_speed = 0;


void subCallBack(const geometry_msgs::Vector3::ConstPtr& msg)
{
    //ROS_WARN("Get Move CMD:%f",msg->x);
    image_turn=msg->x;//读取旋转速度
    image_depth=msg->z;//读取深度值
    geometry_msgs::Twist cmd_msg;
  //  float PIDcal(float setpoint,float actual_position,float Kp,float Ki,float Kd,float epsilon,float MAX,float MIN); //PID函数
    speed_x=PIDcal(2.0,image_depth,-0.5,0,0,0.1,0.6,0.05);
    turn_speed=PIDcal(image_width/2,image_turn,0.002,0,0.00001,50,1.0,0.1);

    cmd_msg.linear.x=speed_x;//对平移速度和旋转速度进行赋值
    cmd_msg.angular.z=turn_speed;
    pub.publish(cmd_msg);
   
    
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_to_cmd");
    ros::NodeHandle ndHandle;

    std::string sub_target_pos = "/target_pose";
    std::string pub_cmd = "/cmd_vel";

    ros::param::get("~sub_mosub_target_pos",     sub_target_pos);
    ros::param::get("~pub_cmd",     pub_cmd);
    ros::param::get("~image_width",    image_width);
    ros::param::get("~image_height",    image_height);

    ros::Subscriber sub = ndHandle.subscribe(sub_target_pos, 1, subCallBack);
    pub = ndHandle.advertise<geometry_msgs::Twist>(pub_cmd, 1);
    ros::spin();

    return 0;
}
