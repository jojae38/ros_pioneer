#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <math.h>
#include "geometry_msgs/Twist.h"
#include "std_msgs/Char.h"
#include <time.h>
#include <opencv2/opencv.hpp>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

using namespace std;

const double camera_width = 0.44;//meter
const double camera_length = 0.34;//meter
const double Robot_wheel_r = 0.11;//meter
const double Robot_center_to_camera_center = 0.34;//meter

#define PI 3.141592

enum MODE {Stop,Front,Right,Left,Find_Marker,Position_adjust};
void poseCallback(const nav_msgs::Odometry::ConstPtr &msg);

struct ORDER
{
    double x;
    double y;
    int order_num;
};

class Pioneer
{
    private:
    double speed;
    double angle_speed;
    double current_time;
    int mode;
    geometry_msgs::Twist vel_msg;
    //cv::VideoCapture *capture;
    vector<ORDER> Adjust_Order;
    vector<ORDER> Move_Order;

    public:
    Pioneer();
    ~Pioneer();
    //Robot Movement
    void Get_param();
    bool set_mode(int num);
    bool go_front();
    bool turn_left();
    bool turn_right();
    bool stop();
    bool run_robot(ros::Publisher cmd_vel_pub);

    bool Adjust_Position();
    
    //Camera Part
    bool is_marker_on_sight();
    bool Publish_image();
    bool run_camera();
    

    //Rviz
    
};
  
Pioneer::Pioneer()
{
    
    mode=MODE::Stop;

    vel_msg.linear.y=0;
    vel_msg.linear.z=0;
    vel_msg.angular.x=0;
    vel_msg.angular.y=0;
    //cv::VideoCapture cap(0);
    //*capture=cap;
    // if(!cap.isOpened())
	// 	std::cerr<<"Camera open failed!"<<std::endl;
    // else
    //     ROS_INFO("Camera model [ELP-USBFHD06H-L21] connected");
    ROS_INFO("Starting Pioneer");
}
Pioneer::~Pioneer()
{
    cv::destroyAllWindows();
    ROS_INFO("END Pioneer");
}
void Pioneer::Get_param()
{
    ros::NodeHandle nh_private("~");
    nh_private.param<double>("speed", speed, 0.1); 
    nh_private.param<double>("angle_speed", angle_speed, 0.1);
    angle_speed*=PI;
}
bool Pioneer::go_front()
{
    vel_msg.linear.x=speed;
    vel_msg.angular.z=0;
    return true;
}
bool Pioneer::turn_left()
{
    vel_msg.linear.x=speed;
    vel_msg.angular.z=-angle_speed;
    return true;
}
bool Pioneer::turn_right()
{
    vel_msg.linear.x=speed;
    vel_msg.angular.z=angle_speed;
    return true;
}
bool Pioneer::stop()
{
    vel_msg.linear.x=0;
    vel_msg.angular.z=0;
    return true;
}
bool Pioneer::is_marker_on_sight()
{
    return true;
}
bool Pioneer::set_mode(int num)
{
    if(mode!=num)
    {
        mode=num;
        return true;
    }
    return false;
}
// bool Pioneer::run_camera()
// {
//     cv::Mat frame;
//     	*capture>>frame;
// 	if(frame.empty())
// 	    return -1;
//     cv::namedWindow("frame");
//     cv::moveWindow("frame",10,0);
//     cv::imshow("frame",frame);
//     if(cv::waitKey(10)==27)
//         return 0;
//     return 0;
// }
bool Pioneer::run_robot(ros::Publisher cmd_vel_pub)
{   ros::Rate rate(60);
    while(ros::ok())
    {
        // Pioneer::run_camera();
        if(mode==MODE::Stop)
        {
            Pioneer::stop();
        }
        else if(mode==MODE::Find_Marker)
        {
            // Pioneer::stop();
            // ROS_INFO("STOP");
            //calculate middle pos
            //set to marker location
        }
        else if(mode==MODE::Position_adjust)
        {
            // Pioneer::stop();
            // ROS_INFO("STOP");
        }
        else if(mode==MODE::Front)
        {
            Pioneer::go_front();
        }
        else if(mode==MODE::Left)
        {
            Pioneer::turn_left();
        }
        else if(mode==MODE::Right)
        {
            Pioneer::turn_right();
        }
        cmd_vel_pub.publish(vel_msg);
        rate.sleep();
        ros::spinOnce();
    }
    return true;

}