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

enum MODE {Stop,Front,Right,Left,Back,Find_Marker,Goto_Marker,Position_adjust};
struct ORDER
{
    double x;
    double y;
    int order_num;
};
struct POSITION
{
    double x;
    double y;
    double th;
};
struct COLOR{
    int R;
    int G;
    int B;
    int offset;
};
class Pioneer
{
    private:
    double speed;
    double angle_speed;
    double current_time;
    int mode;
    struct COLOR MARKER_COLOR;
    struct POSITION ROBOT_POS;
    geometry_msgs::Twist vel_msg;
    //cv::VideoCapture *capture;
    vector<ORDER> Adjust_Order;
    vector<ORDER> Move_Order;
    vector<POSITION> MARKER;

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
    bool back();
    bool run_robot(ros::Publisher cmd_vel_pub,cv::VideoCapture &cap);

    bool Adjust_Position();
    
    //Camera Part
    bool is_marker_on_sight();
    bool is_almost_marker_on_sight();
    bool Publish_image();
    bool run_camera(cv::VideoCapture &cap);
    

    //Rviz
    void visualize();
    
};
  
Pioneer::Pioneer()
{
    mode=MODE::Stop;
    vel_msg.linear.y=0;
    vel_msg.linear.z=0;
    vel_msg.angular.x=0;
    vel_msg.angular.y=0;
    MARKER_COLOR.B=20;
    MARKER_COLOR.G=20;
    MARKER_COLOR.R=235;
    MARKER_COLOR.offset=20;

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
    vel_msg.angular.z=angle_speed;
    return true;
}
bool Pioneer::turn_right()
{
    vel_msg.linear.x=speed;
    vel_msg.angular.z=-angle_speed;
    return true;
}
bool Pioneer::stop()
{
    vel_msg.linear.x=0;
    vel_msg.angular.z=0;
    return true;
}
bool Pioneer::back()
{
    vel_msg.linear.x=-speed;
    vel_msg.angular.z=0;
    return true;
}
bool Pioneer::is_marker_on_sight()//30% of marker is shown
{
    return false;
}
bool Pioneer::is_almost_marker_on_sight()//90% of marker is shown
{
    return false;
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
bool Pioneer::run_camera(cv::VideoCapture &cap)
{
    cv::Mat frame;
    cap>>frame;
	if(frame.empty())
	    return -1;
    
    cv::namedWindow("frame");
    cv::moveWindow("frame",10,0);
    cv::imshow("frame",frame);
    if(cv::waitKey(10)==27)
        return 0;
    return true;
}
void Pioneer::visualize()
{
    cv::Mat map(cv::Size(1001,1001),CV_32FC3,{0,0,0});
    map.convertTo(map,CV_8UC3);
    for(int i=0;i<=10;i++)
    {
        for(int j=0;j<=10;j++)
        {
            for(int k=0;k<map.rows/10;k++)
            {
                map.at<cv::Vec3b>(1+100*i,1+100*j-k)[2]=255;
                map.at<cv::Vec3b>(1+100*i,1+100*j-k)[1]=255;
                map.at<cv::Vec3b>(1+100*i,1+100*j-k)[0]=255;
                map.at<cv::Vec3b>(1+100*i-k,1+100*j)[2]=255;
                map.at<cv::Vec3b>(1+100*i-k,1+100*j)[1]=255;
                map.at<cv::Vec3b>(1+100*i-k,1+100*j)[0]=255;
            }
        }
    }
    
    cv::imshow("map",map);
    cv::waitKey(10);


}
bool Pioneer::run_robot(ros::Publisher cmd_vel_pub,cv::VideoCapture &cap)
{   ros::Rate rate(20);
    while(ros::ok())
    {
        Pioneer::run_camera(cap);
        Pioneer::visualize();
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
        else if(mode==MODE::Goto_Marker)
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
        else if(mode==MODE::Back)
        {
            Pioneer::back();
        }
        cmd_vel_pub.publish(vel_msg);
        rate.sleep();
        ros::spinOnce();
    }
    return true;

}