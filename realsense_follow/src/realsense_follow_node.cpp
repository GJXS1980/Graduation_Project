#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"

#define MIN_DISTANCE 800 //millimetre
#define MAX_DISTANCE 900 //millimetre

double target_x = 0, target_y = 0, target_z = 0;

void callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{

    double x=0, y=0, z=0, qx = 0, qy = 0, qz = 0, qw = 0;
    x = msg-> pose.position.x;
    y = msg-> pose.position.y;
    z = msg-> pose.position.z;
    qx = msg-> pose.orientation.x;
    qy = msg-> pose.orientation.y;
    qz = msg-> pose.orientation.z;
    qw = msg-> pose.orientation.w;

    target_x = x*100;
    target_y = y*100;
    target_z = z*100;

/*
    target_x = msg-> pose.position.x;
    target_y = msg-> pose.position.y;
    target_z = msg-> pose.position.z;
*/

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "realsense_follow");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<geometry_msgs::PoseStamped>("/visp_auto_tracker/object_position",10, callback);
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("realsense_vel", 10);
    geometry_msgs::Twist twist;
    ros::Rate loop(10);
    while(ros::ok()) 
    {
        if(target_z > MAX_DISTANCE)
            {
            twist.linear.x = target_z/10000*0.5;
            } 
        else if(target_z < MIN_DISTANCE)
        	{
        	twist.linear.x = -target_z/10000*0.5;

        	}

        	else
            	{
            	twist.linear.x = 0;
            	}

        if(fabs( atan2(target_x, target_z) ) < 0.1) 
            { //center
            twist.angular.z = 0;
            } 
        else 
            {
            twist.angular.z = atan2(target_x,target_z)*0.5;
            }
            
        pub.publish(twist);
        twist.linear.x = 0;
        twist.linear.y = 0;
        twist.angular.z = 0;
        loop.sleep();
        ros::spinOnce();
    }

    return 0;
}
