#include "robot_control.hpp"
#include "std_msgs/Char.h"

int main(int argc, char ** argv)
{
    ros::init(argc,argv,"key_input");
    ros::NodeHandle n;
    ros::Publisher key_input=n.advertise<std_msgs::Char>("key_input",1);


    ros::Rate rate(5);
    while(ros::ok())
    {
        std_msgs::Char message;
        cin >>message.data;
        if(message.data=='A'||message.data=='a')
            key_input.publish(message);
        else if(message.data=='D'||message.data=='d')
            key_input.publish(message);
        else if(message.data=='S'||message.data=='s')
            key_input.publish(message);
        else if(message.data=='W'||message.data=='w')
            key_input.publish(message);
        else if(message.data=='X'||message.data=='x')
            key_input.publish(message);
        else if(message.data=='Q'||message.data=='q')
            break;
        else
            ROS_WARN("PLEASE TYPE AGAIN!");
        ROS_INFO("SENDING NEW COMMAND");
        ros::spinOnce();
    }
    return 0;
}