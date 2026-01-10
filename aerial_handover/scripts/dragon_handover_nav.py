#!/usr/bin/env python3

import rospy
import math
import tf
import tf.transformations
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from aerial_robot_msgs.msg import FlightNav


class DragonSimpleNav:
    def __init__(self):
        rospy.init_node('dragon_handover_nav', anonymous=False)
        rospy.loginfo("DragonSimpleNav node started successfully")

        # TF listener for coordinate transformations
        self.tf_listener = tf.TransformListener()

        # Subscribe to robot head pose (continuously updated)
        self.robot_head_pose_sub = rospy.Subscriber('/robot_head_pose', PoseStamped, self.robot_head_pose_cb)

        # Subscribe to trigger topic to start transformation and navigation
        self.trigger_sub = rospy.Subscriber('/dragon/trigger_handover', Empty, self.trigger_handover_cb)

        # Subscribe to CoG odometry for navigation feedback
        self.cog_odom_sub = rospy.Subscriber('/dragon/uav/cog/odom', Odometry, self.cog_odom_cb)

        # Subscribe to dragon joint states
        self.joint_states_sub = rospy.Subscriber('/dragon/joint_states', JointState, self.joint_states_cb)

        # Publisher for UAV navigation
        self.uav_nav_pub = rospy.Publisher("/dragon/uav/nav", FlightNav, queue_size=10)

        # Publisher for joint control
        self.joint_control_pub = rospy.Publisher("/dragon/joints_ctrl", JointState, queue_size=10)

        # Current joint states
        self.current_joint_positions = None

        # Current CoG pose
        self.current_cog_pose = None

        # Target CoG pose (calculated from target robot head pose)
        self.target_cog_x = None
        self.target_cog_y = None
        self.target_cog_z = None
        self.target_cog_yaw = None

        # Target robot head pose
        self.target_robot_head_pose = None

        # Joint names we care about (in the order we want)
        self.target_joint_names = [
            'joint1_pitch',
            'joint1_yaw',
            'joint2_pitch',
            'joint2_yaw',
            'joint3_pitch',
            'joint3_yaw'
        ]

        # Link length (approximate distance from CoG to link1/robot head in the desired configuration)
        # This is a simplified approximation - adjust based on actual robot geometry
        self.link_length = rospy.get_param('~link_length', 0.44)

        # Tolerance for checking if desired pose is achieved (radians)
        self.position_tolerance = 0.05

        # Tolerance for navigation (meters)
        self.nav_position_tolerance = 0.1

        # Tolerance for yaw (radians)
        self.nav_yaw_tolerance = 0.1

        # Flag to track if we're currently transforming joints
        self.is_transforming_joints = False

        # Flag to track if we're currently navigating
        self.is_navigating = False

        rospy.loginfo("DragonSimpleNav initialized. Waiting for trigger messages...")

    def robot_head_pose_cb(self, msg):
        """Callback for robot head pose - only stores the latest pose"""
        # Store the target robot head pose (continuously updated)
        self.target_robot_head_pose = msg

    def trigger_handover_cb(self, msg):
        """Callback for trigger topic - starts the transformation and navigation task"""
        if self.target_robot_head_pose is None:
            rospy.logwarn("Cannot start handover: no robot_head_pose received yet")
            return

        if self.is_transforming_joints or self.is_navigating:
            rospy.logwarn("Handover task already in progress, ignoring trigger")
            return

        rospy.loginfo("Trigger received! Starting joint transformation with latest robot_head_pose...")

        # Extract pitch angle from target orientation
        target_quat = [
            self.target_robot_head_pose.pose.orientation.x,
            self.target_robot_head_pose.pose.orientation.y,
            self.target_robot_head_pose.pose.orientation.z,
            self.target_robot_head_pose.pose.orientation.w
        ]
        target_roll, target_pitch, target_yaw = tf.transformations.euler_from_quaternion(target_quat)

        # Update joint1_pitch to match target pitch
        # Keep pitch angles for joint2 and joint3: 0, 0 degrees
        desired_joint_positions = [
            -target_pitch,
            -0.785,
            0.0,
            1.57,
            0.0,
            1.57
        ]

        # Store updated desired positions for later comparison
        self.desired_joint_positions = desired_joint_positions

        # Set the transformation flag
        self.is_transforming_joints = True

        # Create and publish desired joint state
        desired_joint = JointState()
        desired_joint.position = desired_joint_positions

        # Publish the joint command
        self.joint_control_pub.publish(desired_joint)

        rospy.loginfo(f"Published desired joint positions: pitch1={math.degrees(target_pitch):.1f} deg, yaw=[-45, 90, 90] deg, pitch=[0, 0, 0] deg")

    def cog_odom_cb(self, msg):
        """Callback for CoG odometry"""
        self.current_cog_pose = msg.pose.pose

        # Check if navigation is complete
        if self.is_navigating and self.target_robot_head_pose is not None:
            if self.check_navigation_achieved():
                rospy.loginfo("Navigation complete! Robot head (link1) reached target pose.")
                self.is_navigating = False

    def joint_states_cb(self, msg):
        """Callback for dragon joint states"""
        if msg.name and msg.position and len(msg.position) >= 6:
            # Extract only the joint positions we care about
            self.current_joint_positions = []
            for joint_name in self.target_joint_names:
                if joint_name in msg.name:
                    idx = msg.name.index(joint_name)
                    self.current_joint_positions.append(msg.position[idx])
                else:
                    rospy.logwarn_once(f"Joint {joint_name} not found in joint_states message")
                    return

            # Check if joint transformation is complete
            if self.is_transforming_joints:
                if self.check_joint_pose_achieved():
                    rospy.loginfo("Desired joint pose achieved! Joint transformation complete.")
                    self.is_transforming_joints = False

                    # Start navigation after joint transformation
                    self.start_navigation()

    def start_navigation(self):
        if self.target_robot_head_pose is None:
            rospy.logwarn("Cannot start navigation: no target robot head pose available")
            return

        rospy.loginfo("Starting whole body navigation using TF transformation...")
        self.is_navigating = True

        try:
            # Wait for transform to be available
            self.tf_listener.waitForTransform("dragon/root", "dragon/cog", rospy.Time(0), rospy.Duration(4.0))

            # Get the transform from root to cog (cog position in root frame)
            (trans_root_to_cog, rot_root_to_cog) = self.tf_listener.lookupTransform("dragon/root", "dragon/cog", rospy.Time(0))

            # Build transformation matrices
            root_to_cog_matrix = tf.transformations.concatenate_matrices(
                tf.transformations.translation_matrix(trans_root_to_cog),
                tf.transformations.quaternion_matrix(rot_root_to_cog)
            )

            target_root_quat = [
                self.target_robot_head_pose.pose.orientation.x,
                self.target_robot_head_pose.pose.orientation.y,
                self.target_robot_head_pose.pose.orientation.z,
                self.target_robot_head_pose.pose.orientation.w
            ]
            target_root_matrix = tf.transformations.concatenate_matrices(
                tf.transformations.translation_matrix([
                    self.target_robot_head_pose.pose.position.x,
                    self.target_robot_head_pose.pose.position.y,
                    self.target_robot_head_pose.pose.position.z
                ]),
                tf.transformations.quaternion_matrix(target_root_quat)
            )

            # Compute target cog transform: world_to_root * root_to_cog
            target_cog_matrix = tf.transformations.concatenate_matrices(target_root_matrix, root_to_cog_matrix)

            # Extract position and orientation from the result
            target_cog_pos = tf.transformations.translation_from_matrix(target_cog_matrix)
            target_cog_quat = tf.transformations.quaternion_from_matrix(target_cog_matrix)

            # Extract yaw from target cog orientation
            _, _, target_cog_yaw = tf.transformations.euler_from_quaternion(target_cog_quat)

            target_cog_pos_x, target_cog_pos_y, target_cog_pos_z = target_cog_pos

            # Store target CoG pose for navigation checking
            self.target_cog_x = target_cog_pos_x
            self.target_cog_y = target_cog_pos_y
            self.target_cog_z = target_cog_pos_z
            self.target_cog_yaw = target_cog_yaw

            # Create FlightNav message
            nav_msg = FlightNav()
            nav_msg.control_frame = FlightNav.WORLD_FRAME
            nav_msg.target = FlightNav.COG
            nav_msg.pos_xy_nav_mode = FlightNav.POS_MODE
            nav_msg.pos_z_nav_mode = FlightNav.POS_MODE
            nav_msg.yaw_nav_mode = FlightNav.POS_MODE

            # Set target position for CoG
            nav_msg.target_pos_x = self.target_cog_x
            nav_msg.target_pos_y = self.target_cog_y
            nav_msg.target_pos_z = self.target_cog_z
            nav_msg.target_yaw = self.target_cog_yaw

            # Publish navigation command
            self.uav_nav_pub.publish(nav_msg)

            rospy.loginfo(
                f"Published navigation command to CoG: x={self.target_cog_x:.3f}, y={self.target_cog_y:.3f}, z={self.target_cog_z:.3f}, yaw={self.target_cog_yaw:.3f}")
            rospy.loginfo(
                f"Target root pose: x={self.target_robot_head_pose.pose.position.x:.3f}, y={self.target_robot_head_pose.pose.position.y:.3f}, z={self.target_robot_head_pose.pose.position.z:.3f}")

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr(f"TF transformation failed: {e}")
            self.is_navigating = False

    def check_joint_pose_achieved(self):
        """Check if the current joint positions match the desired positions"""
        if self.current_joint_positions is None:
            return False

        for i in range(len(self.desired_joint_positions)):
            error = abs(self.current_joint_positions[i] - self.desired_joint_positions[i])
            if error > self.position_tolerance:
                return False

        return True

    def check_navigation_achieved(self):
        """Check if CoG has reached the target position and orientation"""
        if self.current_cog_pose is None or self.target_cog_x is None:
            return False

        # Get current CoG pose
        current_cog_x = self.current_cog_pose.position.x
        current_cog_y = self.current_cog_pose.position.y
        current_cog_z = self.current_cog_pose.position.z

        current_quat = [
            self.current_cog_pose.orientation.x,
            self.current_cog_pose.orientation.y,
            self.current_cog_pose.orientation.z,
            self.current_cog_pose.orientation.w
        ]
        _, _, current_yaw = tf.transformations.euler_from_quaternion(current_quat)

        # Calculate position error for CoG
        dx = current_cog_x - self.target_cog_x
        dy = current_cog_y - self.target_cog_y
        dz = current_cog_z - self.target_cog_z

        distance = math.sqrt(dx*dx + dy*dy + dz*dz)

        # Normalize yaw error to [-pi, pi]
        yaw_error = self.target_cog_yaw - current_yaw
        while yaw_error > math.pi:
            yaw_error -= 2 * math.pi
        while yaw_error < -math.pi:
            yaw_error += 2 * math.pi

        return distance < self.nav_position_tolerance and abs(yaw_error) < self.nav_yaw_tolerance

    def run(self):
        """Main loop"""
        rospy.spin()


if __name__ == '__main__':
    try:
        nav = DragonSimpleNav()
        nav.run()
    except rospy.ROSInterruptException:
        pass
