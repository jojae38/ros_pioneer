#include "robot_control.hpp"
void poseCallback(const nav_msgs::Odometry::ConstPtr &msg);
void keycallback(const std_msgs::Char::ConstPtr &msg);

Pioneer Pioneer_;
ros::Subscriber pose_vel_sub;
ros::Publisher cmd_vel_pub;
ros::Subscriber key_input;

int main(int argc, char **argv)
{
    ros::init(argc,argv,"Robot_control");
    ros::NodeHandle n;

    pose_vel_sub=n.subscribe("/RosAria/pose",100,poseCallback);
    key_input=n.subscribe("key_input",10,keycallback);
    cmd_vel_pub=n.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel",10);
    Pioneer_.Get_param();
    cv::VideoCapture cap(0);
    if(!cap.isOpened())
		std::cerr<<"Camera open failed!"<<std::endl;
    else
        ROS_INFO("Camera model [ELP-USBFHD06H-L21] connected");
        
    Pioneer_.run_robot(cmd_vel_pub,cap);
    return 0;    
}
void keycallback(const std_msgs::Char::ConstPtr &msg)
{
    
    if(msg->data=='w'||msg->data=='w')
    {
        if(Pioneer_.set_mode(MODE::Front))
        ROS_INFO("CHANGE_MODE -> FRONT");
    }
    else if(msg->data=='s'||msg->data=='S')
    {
        Pioneer_.set_mode(MODE::Stop);
        ROS_INFO("CHANGE_MODE -> STOP");
    }
    else if(msg->data=='d'||msg->data=='D')
    {
        Pioneer_.set_mode(MODE::Right);
        ROS_INFO("CHANGE_MODE -> RIGHT");
    }
    else if(msg->data=='a'||msg->data=='A')
    {
        Pioneer_.set_mode(MODE::Left);
        ROS_INFO("CHANGE_MODE -> LEFT");
    }
    else if(msg->data=='x'||msg->data=='X')
    {
        Pioneer_.set_mode(MODE::Back);
        ROS_INFO("CHANGE_MODE -> BACK");
    }
    ROS_INFO("GET KEY %c",msg->data);
}
void poseCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    // cout << msg->header.frame_id <<endl;
    // cout << msg->header.seq<<endl;
    // cout << msg->header.stamp<<endl;

}