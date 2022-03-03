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
int MARKER_FULL_num= 24000;

#define PI 3.141592
ros::Subscriber pose_vel_sub;
ros::Publisher cmd_vel_pub;
ros::Publisher pose_vel_pub;
ros::Subscriber key_input;
enum MODE {Stop,Front,Right,Left,Back,Init};
enum MARKER_MODE {No_Marker,Find_Marker,Goto_Marker,Position_adjust};
struct POSITION
{
    double x;
    double y;
    double th;
    int order_num;
};
struct COLOR{
    uchar R;
    uchar G;
    uchar B;
    int offset;
};
struct Visual_Map{
    int cross;
    int map_row;
    int map_col;
    int row_block;
    int col_block;
};
class Pioneer
{
    private:
    double speed;
    double angle_speed;
    double current_time;
    int prev_mode;
    int mode;
    int prev_Marker_mode;
    int Marker_mode;
    int MARKER_pixel;
    struct COLOR MARKER_COLOR;
    struct COLOR ROBOT_COLOR_X;
    struct COLOR ROBOT_COLOR_Y;
    struct COLOR ORDER_COLOR;

    struct POSITION ROBOT_POS;
    struct Visual_Map Map;
    geometry_msgs::Twist vel_msg;
    nav_msgs::Odometry odom_msg;

    vector<POSITION> Adjust_Order;
    vector<POSITION> Move_Order;

    POSITION ROBOT;
    vector<POSITION> MARKER;
    vector<POSITION> PATH;

    public:
    Pioneer();
    ~Pioneer();
    //Robot Movement
    void Get_param();
    void set_Position(struct POSITION &pos,double x,double y,double th);
    bool set_mode(int num);
    bool go_front();
    bool turn_left();
    bool turn_right();
    bool stop();
    bool back();
    bool run_robot(cv::VideoCapture &cap);
    bool update_ROBOT_Position(const nav_msgs::Odometry::ConstPtr &msg);
    void add_path_or_marker(vector<POSITION> &Pos, int x,int y,int order_num);
    void set_pose(double x,double y,double th);
    
    //Camera Part
    bool is_marker_on_sight();
    bool Publish_image();
    bool run_camera(cv::VideoCapture &cap);
    
    //visualize
    void set_Visual_map(int cross,int row,int col);
    void set_color(struct COLOR &color,int R,int G,int B,int offset);
    void visualize();
    void draw_robot_at(double x,double y,double th,cv::Mat *map);
    void draw_marker_at(double x,double y,cv::Mat &map,struct COLOR color);
    int convert_world_pos_x(double x);
    int convert_world_pos_y(double y);
};
  
Pioneer::Pioneer()
{
    mode=MODE::Stop;
    prev_mode=MODE::Stop;
    vel_msg.linear.y=0;
    vel_msg.linear.z=0;
    vel_msg.angular.x=0;
    vel_msg.angular.y=0;
    MARKER_pixel=0;
    Pioneer::set_color(MARKER_COLOR,290,-10,-10,100);
    Pioneer::set_color(ROBOT_COLOR_X,100,200,100,0);
    Pioneer::set_color(ROBOT_COLOR_Y,100,100,200,0);
    Pioneer::set_color(ORDER_COLOR,20,20,235,0);
    Pioneer::set_Position(ROBOT,0,0,0);
    Pioneer::set_Visual_map(14,700,700);
    add_path_or_marker(Move_Order,0,5,0);
    add_path_or_marker(Move_Order,0,10,1);
    add_path_or_marker(Move_Order,10,10,2);
    add_path_or_marker(Move_Order,10,3,3);
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
void Pioneer::set_Position(struct POSITION &pos,double x,double y,double th)
{
    pos.x=x;
    pos.y=y;
    pos.th=th;
}
void Pioneer::set_pose(double x,double y,double th)
{
    nav_msgs::Odometry temp=odom_msg;
    temp.pose.pose.position.x=x;
    temp.pose.pose.position.y=y;
    temp.pose.pose.orientation.w=1;
    temp.pose.pose.orientation.z=th;
    pose_vel_pub.publish(temp);

}
bool Pioneer::go_front()
{
    vel_msg.linear.x=speed;
    vel_msg.angular.z=0;
    return true;
}
bool Pioneer::turn_left()
{
    vel_msg.linear.x=0;
    vel_msg.angular.z=angle_speed;
    return true;
}
bool Pioneer::turn_right()
{
    vel_msg.linear.x=0;
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
void Pioneer::add_path_or_marker(vector<POSITION> &Pos, int x,int y,int order_num)
{
    POSITION temp;
    set_Position(temp,x,y,0);
    temp.order_num=order_num;
    Pos.push_back(temp);
}
bool Pioneer::is_marker_on_sight()//30% of marker is shown
{
    if(MARKER_pixel>=MARKER_FULL_num*0.85)//85% of Marker is shown
    {
        Marker_mode=MARKER_MODE::Goto_Marker;
        return true;   
    }
    else if(MARKER_pixel>=MARKER_FULL_num*0.8&&Marker_mode==MARKER_MODE::Goto_Marker)//hysteresis - Goto
    {
        Marker_mode=MARKER_MODE::Goto_Marker;
        return true;
    }
    else if(MARKER_pixel>=MARKER_FULL_num*0.3)//30% of Marker is shown
    {
        Marker_mode=MARKER_MODE::Find_Marker;
        return true;
    }
    else if(MARKER_pixel>=MARKER_FULL_num*0.2&&Marker_mode==MARKER_MODE::Find_Marker)//hysteresis - Find
    {
        Marker_mode=MARKER_MODE::Find_Marker;
        return true;
    }
    else
    {
        Marker_mode=MARKER_MODE::No_Marker;
        return false;
    }
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
    cv::Mat resized_frame;
    cap>>frame;
	if(frame.empty())
	    return -1;
    cv::namedWindow("frame");
    cv::moveWindow("frame",10,0);

    int marker_value=0;
    /*RGB*/
    // for(int i=1;i<frame.rows-1;i++)
    // {
    //     for(int j=1;j<frame.cols-1;j++)
    //     {
    //         int R=frame.at<cv::Vec3b>(i,j)[2];
    //         int G=frame.at<cv::Vec3b>(i,j)[1];
    //         int B=frame.at<cv::Vec3b>(i,j)[0];

    //         if(R>MARKER_COLOR.R-MARKER_COLOR.offset&&G<MARKER_COLOR.G+MARKER_COLOR.offset&&B<MARKER_COLOR.B+MARKER_COLOR.offset)
    //         {
    //             frame.at<cv::Vec3b>(i,j)[2]=255;
    //             frame.at<cv::Vec3b>(i,j)[1]=0;
    //             frame.at<cv::Vec3b>(i,j)[0]=0;
    //             marker_value++;
    //         }    
    //     }
    // }
    /*RGB*/

    cv::Mat mod_frame;
    cv::cvtColor(frame,mod_frame,cv::COLOR_BGR2HSV);
    if(Marker_mode==MARKER_MODE::Find_Marker||Marker_mode==MARKER_MODE::Goto_Marker)
    {
        double x_pos=0;
        double y_pos=0;
        for(int i=1;i<frame.rows-1;i++)
        {
            for(int j=1;j<frame.cols-1;j++)
            {
                int H=mod_frame.at<cv::Vec3b>(i,j)[0];
                int S=mod_frame.at<cv::Vec3b>(i,j)[1];
                int V=mod_frame.at<cv::Vec3b>(i,j)[2];
                
                if((H>170||H<10)&&S>50&&V>30)
                {
                    frame.at<cv::Vec3b>(i,j)={0,0,255};
                    marker_value++;
                    x_pos+=double(i)/double(MARKER_pixel);
                    y_pos+=double(j)/double(MARKER_pixel);
                }    
            }
        }
        ROS_INFO("X_POS = %d",int(x_pos));
        ROS_INFO("Y_POS = %d",int(y_pos));
        MARKER_pixel=marker_value;
    }/*HSV*/
    else
    {
    for(int i=1;i<frame.rows-1;i++)
    {
        for(int j=1;j<frame.cols-1;j++)
        {
            int H=mod_frame.at<cv::Vec3b>(i,j)[0];
            int S=mod_frame.at<cv::Vec3b>(i,j)[1];
            int V=mod_frame.at<cv::Vec3b>(i,j)[2];

            // cout <<H<<" "<<S<<" "<<V<<" "<<endl;
            if((H>170||H<10)&&S>50&&V>30)
            {
                frame.at<cv::Vec3b>(i,j)={0,0,255};
                marker_value++;
            }    
        }
    }
    MARKER_pixel=marker_value;
    // number++;
    }   
    cv::imshow("frame",frame);
    /*HSV*/

    /*RGB RESIZED - if frame is to big to run*/
    // double resize_rate=0.1;
    // cv::resize(frame,resized_frame,cv::Size(),resize_rate,resize_rate);
    // for(int i=1;i<resized_frame.rows-1;i++)
    // {
    //     for(int j=1;j<resized_frame.cols-1;j++)
    //     {
    //         int R=resized_frame.at<cv::Vec3b>(i,j)[2];
    //         int G=resized_frame.at<cv::Vec3b>(i,j)[1];
    //         int B=resized_frame.at<cv::Vec3b>(i,j)[0];

    //         if(R>MARKER_COLOR.R-MARKER_COLOR.offset&&G<MARKER_COLOR.G+MARKER_COLOR.offset&&B<MARKER_COLOR.B+MARKER_COLOR.offset)
    //         {
    //             resized_frame.at<cv::Vec3b>(i,j)[2]=255;
    //             resized_frame.at<cv::Vec3b>(i,j)[1]=0;
    //             resized_frame.at<cv::Vec3b>(i,j)[0]=0;
    //             marker_value++;
    //         }    
    //     }
    // }
    // cv::imshow("frame",resized_frame);
    /*RGB RESIZED*/
    // cout << marker_value<<endl;
    Pioneer::visualize();
    if(cv::waitKey(10)==27)
        return 0;
    return true;
}
void Pioneer::set_color(struct COLOR &color,int R,int G,int B,int offset)
{
    color.R=R;
    color.G=G;
    color.B=B;
    color.offset=offset;
}
void Pioneer::set_Visual_map(int cross,int row,int col)
{
    Map.cross=cross;
    Map.map_row=row;
    Map.map_col=col;
    Map.row_block=Map.map_row/cross;
    Map.col_block=Map.map_col/cross;
}
void Pioneer::visualize()
{
    
    //Make blank map
    
    cv::Mat map(cv::Size(Map.map_row,Map.map_col),CV_8UC3,{0,0,0});
    //Make grid map
    for(int i=1;i<Map.cross;i++)
    {
        for(int k=0;k<Map.map_col;k++)
        {
            map.at<cv::Vec3b>(i*Map.row_block,k)={255,255,255};
        }
        for(int k=0;k<Map.map_row;k++)
        {
            map.at<cv::Vec3b>(k,i*Map.col_block)={255,255,255};
        }
    }
    // Print Marker(whick is found)
    for(int i=0;i<Pioneer::MARKER.size();i++)
    {
        Pioneer::draw_marker_at(convert_world_pos_y(Pioneer::MARKER[i].y),convert_world_pos_x(Pioneer::MARKER[i].x),map,MARKER_COLOR);
    }
    // //Print Order Seq
    for(int i=0;i<Pioneer::Move_Order.size();i++)
    {
        Pioneer::draw_marker_at(convert_world_pos_y(Pioneer::Move_Order[i].y),convert_world_pos_x(Pioneer::Move_Order[i].x),map,ORDER_COLOR);
    }
    // //Print Robot
    Pioneer::draw_robot_at(convert_world_pos_y(ROBOT.y),convert_world_pos_x(ROBOT.x),ROBOT.th,&map);
    cv::namedWindow("map");
    cv::moveWindow("map",865,0);
    cv::imshow("map",map);
}
void Pioneer::draw_robot_at(double x,double y,double th,cv::Mat *map)
{
    double cos_th=cos(ROBOT.th);
    double sin_th=sin(ROBOT.th);
    
    for(int i=-60;i<-5;i++)
    {
        for(int j=-3;j<=3;j++)
        {
            map->at<cv::Vec3b>(int(i*cos_th+j*sin_th+x),int(i*sin_th-j*cos_th+y))={ROBOT_COLOR_X.B,ROBOT_COLOR_X.G,ROBOT_COLOR_X.R};
        }
    }
    for(int i=5;i<60;i++)
    {
        for(int j=-3;j<=3;j++)
        {
            map->at<cv::Vec3b>(int(-i*sin_th-j*cos_th+x),int(i*cos_th-j*sin_th+y))={ROBOT_COLOR_Y.B,ROBOT_COLOR_Y.G,ROBOT_COLOR_Y.R};    
        }
    }
}
void Pioneer::draw_marker_at(double x,double y,cv::Mat &map,struct COLOR color)
{
    for(int i=-5;i<=5;i++)
    {
        for(int j=-5;j<=5;j++)
        {
            map.at<cv::Vec3b>(i+x,j+y)={color.B,color.G,color.R};
        }
    }
}
int Pioneer::convert_world_pos_x(double x)
{
    double world_x=Map.row_block*(x+1);
    return int(world_x);
}
int Pioneer::convert_world_pos_y(double y)
{
    double world_y=Map.col_block*(double(Map.cross-1)-y);
    return int(world_y);
}
bool Pioneer::run_robot(cv::VideoCapture &cap)
{   ros::Rate rate(20);
    while(ros::ok())
    {
        Pioneer::run_camera(cap);
        is_marker_on_sight();
        if(mode!=prev_mode)
        {
            ROS_INFO("MODE_CHANGE TO %d",mode);
            prev_mode=mode;
        }
        // Pioneer::visualize();
        if(Marker_mode==MARKER_MODE::Position_adjust)
        {
            
        }
        else if(Marker_mode==MARKER_MODE::Goto_Marker)
        {
            // Pioneer::stop();
            ROS_INFO("SEE MOST MARKER");
            //calculate middle pos
            //set to marker location
        }
        else if(Marker_mode==MARKER_MODE::Find_Marker)
        {
            // Pioneer::stop();
            ROS_INFO("SEE MARKER");
            //calculate middle pos
            //set to marker location
        }
        else if(Marker_mode==MARKER_MODE::No_Marker)
        {
            // Pioneer::stop();
            // ROS_INFO("STOP");
        }

        if(mode==MODE::Stop)
        {
            Pioneer::stop();
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
        else if(mode==MODE::Init)
        {
            Pioneer::set_pose(0,0,0);
            mode=MODE::Stop;
        }
        cmd_vel_pub.publish(vel_msg);
        rate.sleep();
        ros::spinOnce();
    }
    return true;

}
bool Pioneer::update_ROBOT_Position(const nav_msgs::Odometry::ConstPtr &msg)
{
    odom_msg=*msg;
    ROBOT.x=msg->pose.pose.position.y;
    ROBOT.y=msg->pose.pose.position.x;
    ROBOT.th=(msg->pose.pose.orientation.w)*PI;
    return true;
}
