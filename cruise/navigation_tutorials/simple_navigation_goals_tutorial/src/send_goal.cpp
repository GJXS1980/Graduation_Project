/*
 * send_goal.cpp
 *
 *  Created on: Mar 6, 2019
 *      Author: GJXS
 */

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
/*move_base_msgs::MoveBaseAction
 move_base在world中的目标
*/ 
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>  MoveBaseClient;

int main(int argc, char** argv) 
{
ros::init(argc, argv, "send_goals_node");

/*
创建action客户端；
参数1：action名，参数2：true，不需要手动调用ros::spin()，会在它的线程中自动调用。
*/
MoveBaseClient ac("move_base", true);

// 等待60s使action服务器变得可用
ROS_INFO("Waiting for the move_base action server");
ac.waitForServer(ros::Duration(60));
ROS_INFO("Connected to move base server");

// 发送一个目标点给move_base
//目标的属性设置
move_base_msgs::MoveBaseGoal goal;

goal.target_pose.header.frame_id = "map";
goal.target_pose.header.stamp = ros::Time::now();

/*
goal.target_pose.pose.position.x = 41.7;
goal.target_pose.pose.position.y = 17;
goal.target_pose.pose.orientation.w = 0.00155;
*/

goal.target_pose.pose.position.x = 54.7;
goal.target_pose.pose.position.y = 20.7;
goal.target_pose.pose.orientation.w = 0.00581;



// ROS_INFO("");
ROS_INFO("Sending goal");
ac.sendGoal(goal);

// 等待action返回结果
ac.waitForResult();
if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
ROS_INFO("You have reached the goal!");
else
ROS_INFO("The base failed for some reason");
return 0;
}