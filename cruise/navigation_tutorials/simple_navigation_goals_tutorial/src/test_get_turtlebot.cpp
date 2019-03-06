/*
 * send_goal.cpp
 *
 */

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Point.h>
int point_flag=0;
geometry_msgs::Point target_point;
ros::Subscriber get_target_point_sub;
move_base_msgs::MoveBaseGoal goal;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>  MoveBaseClient;




void get_target_point_cb( const geometry_msgs::Point::ConstPtr& target_msg)
{    
   
      target_point = *target_msg;
printf("aaaaaa %f,%f\n",target_point.x,target_point.y);

MoveBaseClient ac("move_base", true);
// Wait 60 seconds for the action server to become available
ROS_INFO("Waiting for the move_base action server");
ac.waitForServer(ros::Duration(60));
ROS_INFO("Connected to move base server");

// Send a goal to move_base
//目标的属性设置
 //point_flag = target_point;

goal.target_pose.header.frame_id = "map";
goal.target_pose.header.stamp = ros::Time::now();
goal.target_pose.pose.position.x = target_point.x+0.5;
goal.target_pose.pose.position.y = target_point.y-0.5;
// goal.target_pose.pose.position.x = 21.174;
// goal.target_pose.pose.position.y = 10.876;
goal.target_pose.pose.orientation.w = 1;
ROS_INFO("");
ROS_INFO("Sending goal");
// if(target_point.x &&target_point.y)
ac.sendGoal(goal);
// Wait for the action to return
ac.waitForResult();
if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
{
    ROS_INFO("You have reached the goal!");
    sleep(5);
   
  goal.target_pose.pose.position.x =4;
  goal.target_pose.pose.position.y = -4;
  goal.target_pose.pose.orientation.w = 1;
  ac.sendGoal(goal);
  ac.waitForResult();
  if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  ROS_INFO("I'm home! everytning goes fine");
 else
   ROS_INFO("I'm lost in somewhere!");
}

else
ROS_INFO("The base failed for some reason");

}
int main(int argc, char** argv) {
ros::init(argc, argv, "test_get_turtlebot");

 ros::NodeHandle n;
/*X550VC
// create the action client
// true causes the client to spin its own thread
//don't need ros::spin()
创建action客户端，参数1：action名，参数2：true，不需要手动调用ros::spin()，会在它的线程中自动调用。
*/

get_target_point_sub = n.subscribe<geometry_msgs::Point>("/nearest_turtlebot1", 1000, &get_target_point_cb);
//printf("bbbb// %f,%f\n",target_point.x,target_point.y);

ros::spin();
return 0;





}