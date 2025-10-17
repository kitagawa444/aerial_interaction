#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_msgs.msg import Empty
from aerial_robot_msgs.msg import FlightNav

class DroneFollower:
    def __init__(self):
        rospy.init_node('drone_follower')

        #手の位置を格納する変数
        self.hand_position = PoseStamped()
        
        #目標位置を示すself変数self.nav_msgを定義
        self.nav_msg = FlightNav()
        self.nav_msg.control_frame = FlightNav.WORLD_FRAME
        self.nav_msg.target = FlightNav.COG
        self.nav_msg.pos_xy_nav_mode = FlightNav.POS_MODE
        self.nav_msg.pos_z_nav_mode = FlightNav.POS_MODE
        

        #目標位置を発行するノードnav_pub
        self.nav_pub = rospy.Publisher('/quadrotor/uav/nav', FlightNav, queue_size=10)

        #手の位置を購読
        self.hand_sub = rospy.Subscriber("/wrist/mocap/pose", PoseStamped, self.hand_callback)

        #スタートの合図
        rospy.loginfo("Drone FlightNav Follower Node started.")

    def hand_callback(self, msg):
        #手の位置を取得
        self.hand_position = msg
        self.hand_position.header.stamp = rospy.Time.now()
        hand_x = self.hand_position.pose.position.x
        hand_y = self.hand_position.pose.position.y
        hand_z = self.hand_position.pose.position.z

        #ドローンの目標位置に反映
        self.nav_msg.target_pos_x = hand_x
        self.nav_msg.target_pos_y = hand_y
        self.nav_msg.target_pos_z = hand_z + 0.5

        #ドローンの目標位置を発行
        self.nav_pub.publish(self.nav_msg)
        rospy.loginfo("Publish the aim")

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    node = DroneFollower()
    node.run()
