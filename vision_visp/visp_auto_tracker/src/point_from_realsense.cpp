#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"

#include <iostream>
 
using namespace std;

// 接收到订阅的消息后，会进入消息回调函数
void chatterCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    double x=0, y=0, z=0, qx = 0, qy = 0, qz = 0, qw = 0;
    x = msg-> pose.position.x;
    y = msg-> pose.position.y;
    z = msg-> pose.position.z;
    qx = msg-> pose.orientation.x;
    qy = msg-> pose.orientation.y;
    qz = msg-> pose.orientation.z;
    qw = msg-> pose.orientation.w;

    cout << "---" << endl;
    cout << "pose: " << endl;
    cout << "  position: " << endl;
    cout << "\tx: " << x << endl;
    cout << "\ty: " << y << endl;
    cout << "\tz: " << z << endl;
    cout << "  orientation: " << endl;
    cout << "\tx: " << qx << endl;
    cout << "\ty: " << qy << endl;
    cout << "\tz: " << qz << endl;
    cout << "\tw: " << qw << endl;
    cout << "---" << endl;

}

int main(int argc, char **argv)
{
  // 初始化ROS节点
  ros::init(argc, argv, "listener");

  // 创建节点句柄
  ros::NodeHandle n;

  // 创建一个Subscriber，订阅名为chatter的topic，注册回调函数chatterCallback
  ros::Subscriber sub = n.subscribe("/visp_auto_tracker/object_position", 1000, chatterCallback);

  // 循环等待回调函数
  ros::spin();

  return 0;
}
