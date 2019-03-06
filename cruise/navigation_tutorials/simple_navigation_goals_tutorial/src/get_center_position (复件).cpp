#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <mavros_msgs/UavState.h>
#include <ref_generation/ReferenceTarget.h>
#include <uav_state/UavStatePull.h>
#include<cmath>
#include<stdio.h>
static int val=1;
float target_xx=0, target_yy=0, target_zz=0, target_cc=0;


 ros::Publisher target_pub_;
 ros::Publisher turtlebot1_target_pub_;
 ros::Publisher turtlebot2_target_pub_;
 ros::Publisher turtlebot3_target_pub_;
 ros::Publisher turtlebot4_target_pub_;
ros::Subscriber get_center_point_sub;
 ros::ServiceClient pull_state_client_;
mavros_msgs::UavState m_uavstate_;
//center_point_sub_  = it_.subscribe("/camera/image_color", 10, &ImageProc::imgCallback,this);

void get_uavstate()
{
    uav_state::UavStatePull srv;
    if(pull_state_client_.call(srv))
    {
        m_uavstate_ = srv.response.uavstate;
    }
}

void get_center_point_cb( const geometry_msgs::Point::ConstPtr& center)
{    
     
  float cam_x, cam_y, cam_z=0, center_x, center_y;  
  double cam_c;
  float target_x, target_y, target_z, target_c;
  get_uavstate();
  center_x = center->x;//continue when data come
  center_y = center->y;
      while (val<=100)
  { 
    cam_x = center_x*(m_uavstate_.z_pos-0.35)/480;
    cam_y = center_y*(m_uavstate_.z_pos-0.35)/480;
    //cam_z = 0;
    printf("Object center offset is (%f,%f) \n",cam_x,cam_y);
 /* cam_x = abs(2*center_x*atan(2/3)*(m_uavstate_.z_pos-0.35)/640);
    cam_y = abs(2*center_y*atan(1/2)*(m_uavstate_.z_pos-0.35)/480);/ 
*/
    target_x = m_uavstate_.x_pos + cam_x;
    target_y = m_uavstate_.y_pos + cam_y;
    //target_z = m_uavstate_.z_pos + cam_z;
    cam_c = atan2(cam_y, cam_x) ; // radian for camera angle
    target_c = m_uavstate_.c_pos + cam_c;

    target_xx += target_x;
    target_yy += target_y;
    //target_zz += target_z;
    target_cc += target_c;
     //printf("target_zz is %f \n",target_zz);
    ++val;
    break;
  }
 
if (val==101)
{
    // transform target position in local ENU frame
   ref_generation::ReferenceTarget target;
   geometry_msgs::Point turtlebot_target;
     target.x = target_xx/100;
     target.y = target_yy/100;
     turtlebot_target.x = target_xx/100;
     turtlebot_target.y = target_yy/100;
     //target.z = target_zz/100;
     target.z = m_uavstate_.z_pos + cam_z;
     target.c = target_cc/100;
     //printf("target.z is %f \n",target.z);
     // publish to off_mission
     target_pub_.publish(target);

    float target_a,target_b,target_c,target_d,min;
    
    //
    
   target_a = sqrt((target.x-m1)*(target.x-m1) + (target.y-n1)*(target.y-n1));
   target_b = sqrt((target.x-m2)*(target.x-m2) + (target.y-n2)*(target.y-n2));
   target_c = sqrt((target.x-m3)*(target.x-m3) + (target.y-n3)*(target.y-n3));
   target_d = sqrt((target.x-m4)*(target.x-m4) + (target.y-n4)*(target.y-n4));
    min=target_a;
    if(min>target_b)min=target_b；
    if(min>target_c)min=target_c；
    if(min>target_d)min=target_d；

    if(min==target_a)
    {
      turtlebot1_target_pub_.publish(turtlebot_target);
    }
      if(min==target_b)
    {
      turtlebot2_target_pub_.publish(turtlebot_target);
    }
      if(min==target_c)
    {
      turtlebot3_target_pub_.publish(turtlebot_target);
    }
      if(min==target_d)
    {
      turtlebot4_target_pub_.publish(turtlebot_target);
    }



}
   // transform target position in local ENU frame
    // ref_generation::ReferenceTarget target;
    // target.x = m_uavstate_.x_pos + cam_x;
    // target.y = m_uavstate_.y_pos + cam_y;
    // target.z = m_uavstate_.z_pos + cam_z;
    // double cam_c = atan2(cam_y, cam_x) ; // radian for camera angle
    // target.c = m_uavstate_.c_pos + cam_c;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "get_center_position");

        ros::NodeHandle nh_;

    get_center_point_sub = nh_.subscribe<geometry_msgs::Point>("center_point", 1000, &get_center_point_cb);

    target_pub_ = nh_.advertise<ref_generation::ReferenceTarget>("online_target", 100);
    turtlebot1_target_pub_ = nh_.advertise<geometry_msgs::Point>("nearest_turtlebot1", 100);
    turtlebot2_target_pub_ = nh_.advertise<geometry_msgs::Point>("nearest_turtlebot2", 100);
    turtlebot3_target_pub_ = nh_.advertise<geometry_msgs::Point>("nearest_turtlebot3", 100);
    turtlebot4_target_pub_ = nh_.advertise<geometry_msgs::Point>("nearest_turtlebot4", 100);
    pull_state_client_ = nh_.serviceClient<uav_state::UavStatePull>("update_uavstate");

    ros::spin();

    return 0;
}

