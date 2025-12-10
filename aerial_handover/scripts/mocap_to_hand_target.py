#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseStamped

class MocapToHandTarget:
    def __init__(self):
        rospy.init_node('mocap_to_hand_target')

        in_topic  = '/wrist/mocap/pose'
        out_topic = '/hand/target_pose'

        self.pub = rospy.Publisher(out_topic, PoseStamped, queue_size=1)
        self.sub = rospy.Subscriber(in_topic, PoseStamped, self.cb, queue_size=1)

        rospy.loginfo("mocap_to_hand_target: %s -> %s", in_topic, out_topic)

    def cb(self, msg):
        # ここで frame_id を必要に応じて補正してもいい
        # msg.header.frame_id = 'world'
        self.pub.publish(msg)

    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = MocapToHandTarget()
        node.spin()
    except rospy.ROSInterruptException:
        pass
