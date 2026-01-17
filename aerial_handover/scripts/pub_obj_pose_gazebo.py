#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped

class ObjectPosePublisher:
    def __init__(self):
        rospy.init_node('object_pose_publisher')

        # Gazebo 上の object モデル名
        self.model_name = rospy.get_param('~model_name', 'bottle')

        # Publish the pose of the object
        self.pub = rospy.Publisher('object_pose', PoseStamped, queue_size=1)

        # Subscribe to Gazebo model states
        self.sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback, queue_size=1)

    def model_states_callback(self, msg):
        try:
            # Find the index of the model
            index = msg.name.index(self.model_name)
            pose = msg.pose[index]

            # Create a PoseStamped message
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = rospy.Time.now()
            pose_stamped.header.frame_id = "world"
            pose_stamped.pose = pose

            # Publish the pose
            self.pub.publish(pose_stamped)
        except ValueError:
            rospy.logwarn("Model '%s' not found in Gazebo model states", self.model_name)

if __name__ == '__main__':
    try:
        ObjectPosePublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
