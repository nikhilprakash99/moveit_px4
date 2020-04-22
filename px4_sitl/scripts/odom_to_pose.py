#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry as odom
from geometry_msgs.msg import PoseWithCovarianceStamped as pose

def callback(data):
    cur_pose.pose = data.pose
    cur_pose.header = data.header
    pub.publish(cur_pose)

if __name__ == '__main__':

    rospy.init_node('odom_to_pose', anonymous=True)
    rospy.Subscriber("from", odom, callback)
    pub = rospy.Publisher('to', pose, queue_size=10)
    cur_pose=pose()
    rospy.spin()