#include "robot_control.hpp"
void poseCallback(const nav_msgs::Odometry::ConstPtr &msg);
void keycallback(const std_msgs::Char::ConstPtr &msg);

ros::Subscriber pose_vel_sub;
ros::Publisher cmd_vel_pub;
ros::Subscriber key_input;
FLAG temp={true,false,false,false,false,false};
int main(int argc, char **argv)
{
    ros::init(argc,argv,"Robot_control");
    ros::NodeHandle n;
    pose_vel_sub=n.subscribe("/RosAria/pose",100,poseCallback);
    key_input=n.subscribe("key_input",10,keycallback);
    cmd_vel_pub=n.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel",10);

    Pioneer Pioneer_;
    Pioneer_.run_robot(temp,cmd_vel_pub);
    return 0;    
}
void keycallback(const std_msgs::Char::ConstPtr &msg)
{
    
    if(msg->data=='w'||msg->data=='w')
    {
        temp.front=true;
        temp.stop=false;
        temp.left=false;
        temp.right=false;
        temp.Position_adjust=false;
        temp.See_Marker=false;
    }
    else if(msg->data=='s'||msg->data=='S')
    {
        temp.front=true;
        temp.stop=false;
        temp.left=false;
        temp.right=false;
        temp.Position_adjust=false;
        temp.See_Marker=false;
    }
    else if(msg->data=='d'||msg->data=='D')
    {
        temp.front=true;
        temp.stop=false;
        temp.left=false;
        temp.right=true;
        temp.Position_adjust=false;
        temp.See_Marker=false;
    }
    else if(msg->data=='a'||msg->data=='A')
    {
        temp.front=false;
        temp.stop=false;
        temp.left=true;
        temp.right=false;
        temp.Position_adjust=false;
        temp.See_Marker=false;
    }
    else
    {
        temp.front=false;
        temp.stop=true;
        temp.left=false;
        temp.right=false;
        temp.Position_adjust=false;
        temp.See_Marker=false;
    }

}
void poseCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    cout << msg->header.frame_id <<endl;
    cout << msg->header.seq<<endl;
    cout << msg->header.stamp<<endl;

}