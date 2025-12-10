#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState


class HandGazeboFollower:
    def __init__(self):
        rospy.init_node('hand_gazebo_from_interactive_marker')

        # Gazebo 上の hand モデル名（task.launch で -model hand になっている）
        self.model_name = rospy.get_param('~model_name', 'hand')

        # 手の目標位置を受け取るトピック
        # まずは /desired_3D_pose でも /hand/target_pose でも好きな方でOK
        self.pose_topic = rospy.get_param('~pose_topic', '/hand/target_pose')

        rospy.loginfo("Waiting for /gazebo/set_model_state service...")
        rospy.wait_for_service('/gazebo/set_model_state')
        self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        rospy.loginfo("Connected to /gazebo/set_model_state")

        self.sub = rospy.Subscriber(self.pose_topic, PoseStamped,
                                    self.pose_callback, queue_size=1)

    def pose_callback(self, msg):
        state = ModelState()
        state.model_name = self.model_name
        state.pose = msg.pose
        # world 座標系でセット
        state.reference_frame = "world"

        try:
            self.set_model_state(state)
        except rospy.ServiceException as e:
            rospy.logwarn("Failed to call set_model_state: %s", e)


if __name__ == '__main__':
    try:
        HandGazeboFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
