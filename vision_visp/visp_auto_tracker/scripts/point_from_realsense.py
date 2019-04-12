#!/usr/bin/env python 
# -*- coding: utf-8 -*- 

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_msgs.msg import Float64  
import numpy

def callback(pose): 
    global x, y, z, qx, qy, qz, qw
    x = pose.pose.position.x
    y = pose.pose.position.y
    z = pose.pose.position.z 
    qx = pose.pose.orientation.x
    qy = pose.pose.orientation.y
    qz = pose.pose.orientation.z
    qw = pose.pose.orientation.w
    print '---\n pose: \n  position: \n\t x: %.12f \n\t y: %.12f\n\t z: %.12f\n  orientation: \n\t x: %.12f\n\t y: %.12f\n\t z: %.12f\n\t w: %.12f \n---' %(x, y, z, qx, qy, qz, qw)

def listener(): 
    rospy.init_node('listener', anonymous=True) #初始化节点，节点名称
    pose=PoseStamped()     
    rospy.Subscriber('/visp_auto_tracker/object_position', PoseStamped, callback,queue_size = 1)
    rospy.spin()

if __name__ == "__main__":
    listener()

