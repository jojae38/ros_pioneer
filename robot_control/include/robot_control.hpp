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
void poseCallback(const nav_msgs::Odometry::ConstPtr &msg);

struct FLAG{
    bool stop;
    bool front;
    bool right;
    bool left;
    bool See_Marker;
    bool Position_adjust;
};
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
   
    geometry_msgs::Twist vel_msg;
    cv::VideoCapture *capture;
    FLAG Control_Flag;
    vector<ORDER> Adjust_Order;
    vector<ORDER> Move_Order;

    public:
    Pioneer();
    ~Pioneer();
    //Robot Movement
    bool go_front();
    bool turn_left();
    bool turn_right();
    bool stop();
    bool run_robot(FLAG flag,ros::Publisher cmd_vel_pub);
    void set_flag(FLAG *flag);

    bool Adjust_Position();
    
    //Camera Part
    bool is_marker_on_sight();
    bool Publish_image();
    bool run_camera();
    

    //Rviz
    
};
  
Pioneer::Pioneer()
{
    
    
    Control_Flag={true,false,false,false,false,false};
    ros::NodeHandle nh_private("~");
    nh_private.param<double>("speed", speed, 0.1); 
    nh_private.param<double>("angle_speed", angle_speed, 0.1);
    angle_speed*=PI;

    vel_msg.linear.y=0;
    vel_msg.linear.z=0;
    vel_msg.angular.x=0;
    vel_msg.angular.y=0;
    cv::VideoCapture cap(0);
    *capture=cap;
    if(!cap.isOpened())
		std::cerr<<"Camera open failed!"<<std::endl;
    else
        ROS_INFO("Camera model [ELP-USBFHD06H-L21] connected");
    ROS_INFO("Starting Pioneer");
}
Pioneer::~Pioneer()
{
    cv::destroyAllWindows();
    ROS_INFO("END Pioneer");
}
bool Pioneer::go_front()
{
    vel_msg.linear.x=speed;
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
bool Pioneer::run_camera()
{
    cv::Mat frame;
    	*capture>>frame;
	if(frame.empty())
	    return -1;
    cv::namedWindow("frame");
    cv::moveWindow("frame",10,0);
    cv::imshow("frame",frame);
    if(cv::waitKey(10)==27)
        return 0;
    return 0;
}

void Pioneer::set_flag(FLAG *flag)
{
    Control_Flag=*flag;
}
bool Pioneer::run_robot(FLAG flag,ros::Publisher cmd_vel_pub)
{   ros::Rate rate(60);
    while(ros::ok())
    {
        Pioneer::run_camera();
        if(Control_Flag.stop)
        {
            Pioneer::stop();
        }
        else if(Control_Flag.See_Marker)
        {
            Pioneer::stop();
            //calculate middle pos
            //set to marker location
        }
        else if(Control_Flag.Position_adjust)
        {
            Pioneer::stop();
        }
        else if(Control_Flag.front)
        {
            Pioneer::go_front();
        }
        else if(Control_Flag.left)
        {
            Pioneer::turn_left();
        }
        else if(Control_Flag.right)
        {
            Pioneer::turn_right();
        }
        else
        {
            Pioneer::stop();
        }
        cmd_vel_pub.publish(vel_msg);
        rate.sleep();
        ros::spinOnce();
    }
    return true;

}