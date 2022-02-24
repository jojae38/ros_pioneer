#include "robot_control.hpp"

int main(int argc, char **argv)
{
    ros::init(argc,argv,"Robot_control");
    Pioneer Pioneer_;
    Pioneer_.run_robot();
    return 0;    
}