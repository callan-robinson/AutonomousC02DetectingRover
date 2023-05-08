#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, Point

def talker():
    pub = rospy.Publisher('my_pose', Point, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(7.5) # 10hz
    while not rospy.is_shutdown():
        slam_pose = rospy.wait_for_message('slam_out_pose', PoseStamped, timeout=1)
        x = slam_pose.pose.position.x
        y = slam_pose.pose.position.y
        pub.publish(Point(x, y, 0.0))
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass