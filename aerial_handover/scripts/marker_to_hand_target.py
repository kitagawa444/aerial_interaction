#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import PoseStamped

class MarkerToHandTarget:
    def __init__(self):
        rospy.init_node('marker_to_hand_target')

        in_topic = rospy.get_param('~in_topic', '/desired_3D_pose')
        out_topic = rospy.get_param('~out_topic', '/hand/target_pose')

        self.sub = rospy.Subscriber('/desired_3D_pose', PoseStamped, self.cb, queue_size=1)
        self.pub = rospy.Publisher('/hand/target_pose', PoseStamped, queue_size=1)

    def cb(self, msg):
        self.pub.publish(msg)

if __name__ == '__main__':
    try:
        MarkerToHandTarget()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass