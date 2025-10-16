#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseStamped

class DroneFollower:
    def __init__(self):
        rospy.init_node('drone_follower')

        #手の位置を購読
        rospy.Subscriber("/wrist/mocap/pose", PoseStamped, self.hand_callback)

        #ドローンの目標ポーズを発行
        self.pub = rospy.Publisher('/drone/setpoint_position/local', PoseStamped, queue_size=10)

        #ドローンの目標姿勢（初期化）
        self.target_pose = PoseStamped()
        self.target_pose.header.frame_id = "world"

        #姿勢は固定（水平姿勢）
        self.target_pose.pose.orientation.x = 0.0
        self.target_pose.pose.orientation.y = 0.0
        self.target_pose.pose.orientation.z = 0.0
        self.target_pose.pose.orientation.w = 1.0

        rospy.loginfo("Drone Position Follower Node started.")

    def hand_callback(self, msg):
        #手の位置を取得して、ドローン目標に反映
        self.target_pose.header.stamp = rospy.Time.now()

        self.target_pose.pose.position.x =  msg.pose.position.x
        self.target_pose.pose.position.y = msg.pose.position.y
        self.target_pose.pose.position.z = msg.pose.position.z + 0.5

        #姿勢は固定のまま
        self.pub.publish(self.target_pose)

    def chase_hand(self):
        

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    node = DroneFollower()
    node.run()
