#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Empty

# new imports
from gazebo_msgs.srv import SetModelState, SetModelStateRequest
from gazebo_msgs.msg import ModelState

# Try import service type; if unavailable, node will log warning and skip service calls.
try:
    from gazebo_ros_link_attacher.srv import Attach, AttachRequest
    _HAS_ATTACH_SRV = True
except Exception:
    rospy.logwarn("gazebo_ros_link_attacher.srv.Attach not found. Service calls will be skipped.")
    _HAS_ATTACH_SRV = False


class HandGraspSnakeBite:
    """Publish object pose from Gazebo and call attach/detach on handgrasp events."""
    def __init__(self):
        rospy.init_node('hand_grasp_snake_bite')

        # model/link params (can be overridden by ROS params)
        self.hand_model = rospy.get_param('~hand_model', 'hand')
        self.hand_link = rospy.get_param('~hand_link', 'hand_link')
        self.snake_model = rospy.get_param('~snake_model', 'dragon')
        self.snake_link = rospy.get_param('~snake_link', 'root')
        self.object_model = rospy.get_param('~object_model', 'bottle')
        self.object_link = rospy.get_param('~object_link', 'bottle_link')

        # service/topic names
        self.attach_service_name = rospy.get_param('~attach_service_name', '/link_attacher_node/attach')
        self.detach_service_name = rospy.get_param('~detach_service_name', '/link_attacher_node/detach')
        self.handgrasp_topic = rospy.get_param('~handgrasp_topic', '/handgrasp')
        self.snakebite_topic = rospy.get_param('~snakebite_topic', '/snakebite')

        # Publishers / Subscribers
        self.pose_pub = rospy.Publisher('/object_pose', PoseStamped, queue_size=1)
        self.model_states_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback, queue_size=1)
        self.handgrasp_sub = rospy.Subscriber(self.handgrasp_topic, Empty, self.handgrasp_callback, queue_size=1)
        self.snakebite_sub = rospy.Subscriber(self.snakebite_topic, Empty, self.snakebite_callback, queue_size=1)

        # store latest poses by model name
        self.model_poses = {}

        # tracking / pause flags
        self.tracking = False
        self.paused = False
        self.object_in_hand = None
        self.track_timer = None

        # service proxies
        self.detach_proxy = None
        self.attach_proxy = None
        self.set_model_state_proxy = None
        if _HAS_ATTACH_SRV:
            try:
                rospy.wait_for_service(self.detach_service_name, timeout=5.0)
                rospy.wait_for_service(self.attach_service_name, timeout=5.0)
                self.detach_proxy = rospy.ServiceProxy(self.detach_service_name, Attach)
                self.attach_proxy = rospy.ServiceProxy(self.attach_service_name, Attach)
                rospy.loginfo("Service proxies ready: %s, %s", self.detach_service_name, self.attach_service_name)
            except Exception as e:
                rospy.logwarn("Could not connect to attach/detach services: %s", e)
                self.detach_proxy = None
                self.attach_proxy = None

        # set_model_state service proxy (optional but preferred)
        try:
            rospy.wait_for_service('/gazebo/set_model_state', timeout=5.0)
            self.set_model_state_proxy = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            rospy.loginfo("Connected to /gazebo/set_model_state service")
        except Exception:
            rospy.logwarn("Could not connect to /gazebo/set_model_state service; will skip SetModelState calls")

    def model_states_callback(self, msg):
        try:
            # store poses for lookup
            for name, pose in zip(msg.name, msg.pose):
                self.model_poses[name] = pose

            # keep publishing object pose as existing behavior
            if self.object_model in self.model_poses:
                pose = self.model_poses[self.object_model]
                ps = PoseStamped()
                ps.header.stamp = rospy.Time.now()
                ps.header.frame_id = "world"
                ps.pose = pose
                self.pose_pub.publish(ps)
        except Exception as e:
            rospy.logwarn_throttle(10, "Error in model_states_callback: %s", e)

    # utility: pose -> 4x4 matrix
    def pose_to_matrix(self, pose):
        import tf
        pos = (pose.position.x, pose.position.y, pose.position.z)
        ori = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        trans = tf.transformations.translation_matrix(pos)
        rot = tf.transformations.quaternion_matrix(ori)
        return tf.transformations.concatenate_matrices(trans, rot)

    # utility: matrix -> Pose
    def matrix_to_pose(self, mat):
        import tf
        trans = tf.transformations.translation_from_matrix(mat)
        quat = tf.transformations.quaternion_from_matrix(mat)
        p = Pose()
        p.position.x = trans[0]
        p.position.y = trans[1]
        p.position.z = trans[2]
        p.orientation.x = quat[0]
        p.orientation.y = quat[1]
        p.orientation.z = quat[2]
        p.orientation.w = quat[3]
        return p

    # start periodic tracking (update object's world pose to follow hand + relative transform)
    def start_tracking(self, rate_hz=20.0):
        if self.tracking:
            return
        self.tracking = True
        self.paused = False
        period = rospy.Duration(1.0 / rate_hz)
        self.track_timer = rospy.Timer(period, self._track_timer_cb)
        rospy.loginfo("Started object tracking at %s Hz", rate_hz)

    def stop_tracking(self):
        if self.track_timer is not None:
            self.track_timer.shutdown()
            self.track_timer = None
        self.tracking = False
        rospy.loginfo("Stopped object tracking")

    def _track_timer_cb(self, event):
        if self.paused or self.object_in_hand is None:
            return
        # need current hand pose
        hand_pose = self.model_poses.get(self.hand_model)
        if hand_pose is None:
            rospy.logwarn_throttle(5, "Hand pose missing; cannot update object state")
            return
        try:
            # compute world = hand * object_in_hand
            mat_hand = self.pose_to_matrix(hand_pose)
            mat_rel = self.pose_to_matrix(self.object_in_hand)
            mat_world = self._mat_mult(mat_hand, mat_rel)
            new_pose = self.matrix_to_pose(mat_world)
            self._call_set_model_state(self.object_model, new_pose)
        except Exception as e:
            rospy.logwarn_throttle(5, "Error updating object state: %s", e)

    def _mat_mult(self, a, b):
        import numpy as np
        return (a @ b)

    def _call_set_model_state(self, model_name, pose):
        if self.set_model_state_proxy is None:
            rospy.logwarn_throttle(10, "No set_model_state proxy; skipping pose set for %s", model_name)
            return
        try:
            req = SetModelStateRequest()
            ms = ModelState()
            ms.model_name = model_name
            ms.pose = pose
            ms.reference_frame = "world"
            req.model_state = ms
            resp = self.set_model_state_proxy(ms)
            rospy.loginfo_throttle(5, "SetModelState for %s: %s", model_name, getattr(resp, 'status_message', 'no_msg'))
        except Exception as e:
            rospy.logwarn_throttle(5, "SetModelState call failed: %s", e)

    def handgrasp_callback(self, msg):
        rospy.loginfo("Received handgrasp on %s", self.handgrasp_topic)

        # compute and store relative pose object_in_hand at the moment of detach
        hand_pose = self.model_poses.get(self.hand_model)
        object_pose = self.model_poses.get(self.object_model)
        if hand_pose is None or object_pose is None:
            rospy.logwarn("Cannot compute relative pose: missing hand or object pose")
        else:
            try:
                mat_hand = self.pose_to_matrix(hand_pose)
                mat_obj = self.pose_to_matrix(object_pose)
                # rel = inverse(hand) * obj
                import tf
                mat_hand_inv = tf.transformations.inverse_matrix(mat_hand)
                mat_rel = mat_hand_inv @ mat_obj
                self.object_in_hand = self.matrix_to_pose(mat_rel)
                rospy.loginfo("Saved object relative pose in hand frame")
            except Exception as e:
                rospy.logwarn("Failed to compute/save relative pose: %s", e)

        # Detach first (existing behavior)
        if not _HAS_ATTACH_SRV or (self.detach_proxy is None and self.attach_proxy is None):
            rospy.logwarn("Attach/detach services not available, skipping service calls.")
            return

        detach_req = AttachRequest()
        detach_req.model_name_1 = self.snake_model
        detach_req.link_name_1 = self.snake_link
        detach_req.model_name_2 = self.object_model
        detach_req.link_name_2 = self.object_link

        try:
            if self.detach_proxy is not None:
                det_resp = self.detach_proxy(detach_req)
                rospy.loginfo("Detach service response: %s", det_resp)
            else:
                rospy.logwarn("Detach proxy not available.")
        except Exception as e:
            rospy.logwarn("Detach service call failed: %s", e)

        # after detach, start applying set_model_state updates so object follows hand+relative
        if self.object_in_hand is not None:
            self.paused = False
            self.start_tracking(rate_hz=20.0)
            # apply one immediate set to ensure placement
            if self.set_model_state_proxy is not None and hand_pose is not None:
                try:
                    mat_hand = self.pose_to_matrix(hand_pose)
                    mat_rel = self.pose_to_matrix(self.object_in_hand)
                    new_mat = mat_hand @ mat_rel
                    new_pose = self.matrix_to_pose(new_mat)
                    self._call_set_model_state(self.object_model, new_pose)
                except Exception as e:
                    rospy.logwarn("Immediate SetModelState failed: %s", e)

    def snakebite_callback(self, msg):
        rospy.loginfo("Received snakebite on %s", self.snakebite_topic)

        # when snakebite (attach) arrives, pause tracking to avoid race with attach
        self.paused = True
        # stop tracking to avoid overwriting attachment (optional)
        self.stop_tracking()

        if not _HAS_ATTACH_SRV or (self.detach_proxy is None and self.attach_proxy is None):
            rospy.logwarn("Attach/detach services not available, skipping service calls.")
            return

        attach_req = AttachRequest()
        attach_req.model_name_1 = self.snake_model
        attach_req.link_name_1 = self.snake_link
        attach_req.model_name_2 = self.object_model
        attach_req.link_name_2 = self.object_link

        # Attach
        try:
            if self.attach_proxy is not None:
                att_resp = self.attach_proxy(attach_req)
                rospy.loginfo("Attach service response: %s", att_resp)
            else:
                rospy.logwarn("Attach proxy not available.")
        except Exception as e:
            rospy.logwarn("Attach service call failed: %s", e)


if __name__ == '__main__':
    try:
        HandGraspSnakeBite()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass