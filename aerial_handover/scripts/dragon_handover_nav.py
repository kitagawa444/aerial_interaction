#!/usr/bin/env python3

import rospy
import math
import tf
import tf.transformations
import geometry_msgs.msg
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from aerial_robot_msgs.msg import FlightNav
from visualization_msgs.msg import Marker


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

        # Publisher for visualization markers
        self.marker_pub = rospy.Publisher("/dragon/intermediate_pose_marker", Marker, queue_size=10)

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

        # Human center position (point H)
        self.human_center_x = None
        self.human_center_y = None

        # Intermediate target for safe approach
        self.intermediate_cog_x = None
        self.intermediate_cog_y = None
        self.intermediate_cog_z = None
        self.intermediate_cog_yaw = None

        # Navigation stage: 'intermediate', 'arc_motion', or 'final'
        self.navigation_stage = None

        # Arc motion waypoints (list of CoG poses)
        self.arc_waypoints = []
        self.current_waypoint_index = 0

        # Number of waypoints for arc motion
        self.num_arc_waypoints = rospy.get_param('~num_arc_waypoints', 10)

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

        # Safe distance from human center (meters)
        self.safe_distance = rospy.get_param('~safe_distance', 1.0)

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

        # Calculate and publish visualization markers early
        # Calculate human center (point H)
        # Point H is along the opposite direction (180 degrees) from where the robot head is pointing
        direction_x = -math.cos(target_yaw)
        direction_y = -math.sin(target_yaw)
        self.human_center_x = self.target_robot_head_pose.pose.position.x + direction_x * self.safe_distance
        self.human_center_y = self.target_robot_head_pose.pose.position.y + direction_y * self.safe_distance

        rospy.loginfo(f"Human center (H) calculated at: x={self.human_center_x:.3f}, y={self.human_center_y:.3f}")

        # Publish visualization markers
        self.publish_human_center_marker(self.human_center_x, self.human_center_y, self.target_robot_head_pose.pose.position.z)
        self.publish_safety_circle_marker(self.human_center_x, self.human_center_y,
                                          self.target_robot_head_pose.pose.position.z, self.safe_distance)
        self.publish_final_head_pose_marker(
            self.target_robot_head_pose.pose.position.x,
            self.target_robot_head_pose.pose.position.y,
            self.target_robot_head_pose.pose.position.z,
            target_yaw
        )

    def cog_odom_cb(self, msg):
        """Callback for CoG odometry"""
        self.current_cog_pose = msg.pose.pose

        # Check if navigation is complete
        if self.is_navigating and self.target_robot_head_pose is not None:
            if self.check_navigation_achieved():
                rospy.loginfo("Navigation complete! Robot reached final handover pose safely.")
                self.is_navigating = False
                self.navigation_stage = None

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

        rospy.loginfo("Starting whole body navigation with safe approach strategy...")
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

            # Store final target CoG pose for navigation checking
            self.target_cog_x = target_cog_pos_x
            self.target_cog_y = target_cog_pos_y
            self.target_cog_z = target_cog_pos_z
            self.target_cog_yaw = target_cog_yaw

            # Calculate human center (point H)
            # Point H is along the opposite direction (180 degrees) from where the robot head is pointing
            target_roll, target_pitch, target_yaw_head = tf.transformations.euler_from_quaternion(target_root_quat)

            # Direction vector from robot head pose (reversed 180 degrees)
            direction_x = -math.cos(target_yaw_head)
            direction_y = -math.sin(target_yaw_head)

            # Human center H is at safe_distance along this reversed direction (2D plane)
            self.human_center_x = self.target_robot_head_pose.pose.position.x + direction_x * self.safe_distance
            self.human_center_y = self.target_robot_head_pose.pose.position.y + direction_y * self.safe_distance

            # Calculate intermediate point M
            # M is the intersection of line from current CoG to H with the safety circle
            if self.current_cog_pose is None:
                rospy.logwarn("Current CoG pose not available, cannot calculate intermediate point")
                self.is_navigating = False
                return

            current_cog_x = self.current_cog_pose.position.x
            current_cog_y = self.current_cog_pose.position.y

            # Vector from current CoG to human center
            dx = self.human_center_x - current_cog_x
            dy = self.human_center_y - current_cog_y
            distance_to_human = math.sqrt(dx*dx + dy*dy)

            if distance_to_human < self.safe_distance:
                rospy.logwarn(f"Robot is too close to human center ({distance_to_human:.3f}m < {self.safe_distance:.3f}m)")
                # Still proceed but be careful

            # Normalize direction vector
            if distance_to_human > 1e-6:
                dx_norm = dx / distance_to_human
                dy_norm = dy / distance_to_human
            else:
                rospy.logerr("Current CoG is at human center, cannot calculate safe approach")
                self.is_navigating = False
                return

            # Point M is at distance (distance_to_human - safe_distance) from current CoG
            # Or equivalently, at safe_distance from H along the line from CoG to H
            intermediate_x = self.human_center_x - dx_norm * self.safe_distance
            intermediate_y = self.human_center_y - dy_norm * self.safe_distance

            # Intermediate orientation: pointing from H to M (since root link x-axis points from head to body)
            intermediate_yaw = math.atan2(-dy_norm, -dx_norm)

            # Calculate intermediate CoG pose
            # Create intermediate root pose
            intermediate_root_matrix = tf.transformations.concatenate_matrices(
                tf.transformations.translation_matrix([intermediate_x, intermediate_y, self.target_robot_head_pose.pose.position.z]),
                tf.transformations.quaternion_matrix(
                    tf.transformations.quaternion_from_euler(0, 0, intermediate_yaw)
                )
            )

            # Compute intermediate cog transform: intermediate_root * root_to_cog
            intermediate_cog_matrix = tf.transformations.concatenate_matrices(intermediate_root_matrix, root_to_cog_matrix)

            # Extract intermediate CoG pose
            intermediate_cog_pos = tf.transformations.translation_from_matrix(intermediate_cog_matrix)
            intermediate_cog_quat = tf.transformations.quaternion_from_matrix(intermediate_cog_matrix)
            _, _, intermediate_cog_yaw = tf.transformations.euler_from_quaternion(intermediate_cog_quat)

            self.intermediate_cog_x = intermediate_cog_pos[0]
            self.intermediate_cog_y = intermediate_cog_pos[1]
            self.intermediate_cog_z = intermediate_cog_pos[2]
            self.intermediate_cog_yaw = intermediate_cog_yaw

            rospy.loginfo(
                f"Intermediate point (M) head pose: x={intermediate_x:.3f}, y={intermediate_y:.3f}, yaw={math.degrees(intermediate_yaw):.1f} deg")
            rospy.loginfo(
                f"Intermediate CoG pose: x={self.intermediate_cog_x:.3f}, y={self.intermediate_cog_y:.3f}, z={self.intermediate_cog_z:.3f}, yaw={math.degrees(self.intermediate_cog_yaw):.1f} deg")

            # Publish marker for intermediate head pose visualization
            self.publish_intermediate_pose_marker(intermediate_x, intermediate_y, self.target_robot_head_pose.pose.position.z, intermediate_yaw)

            # Generate arc motion waypoints
            self.generate_arc_waypoints(root_to_cog_matrix)
            self.current_waypoint_index = 0

            # Start with intermediate navigation
            self.navigation_stage = 'intermediate'

            # Create FlightNav message for intermediate pose
            nav_msg = FlightNav()
            nav_msg.control_frame = FlightNav.WORLD_FRAME
            nav_msg.target = FlightNav.COG
            nav_msg.pos_xy_nav_mode = FlightNav.POS_MODE
            nav_msg.pos_z_nav_mode = FlightNav.POS_MODE
            nav_msg.yaw_nav_mode = FlightNav.POS_MODE

            # Set intermediate target position for CoG
            nav_msg.target_pos_x = self.intermediate_cog_x
            nav_msg.target_pos_y = self.intermediate_cog_y
            nav_msg.target_pos_z = self.intermediate_cog_z
            nav_msg.target_yaw = self.intermediate_cog_yaw

            # Publish navigation command
            self.uav_nav_pub.publish(nav_msg)

            rospy.loginfo(f"Stage 1: Navigating to intermediate safe position...")

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr(f"TF transformation failed: {e}")
            self.is_navigating = False

    def generate_arc_waypoints(self, root_to_cog_matrix):
        """Generate waypoints for arc motion from intermediate to final pose"""
        # Calculate angles for intermediate and final positions relative to human center H
        intermediate_angle = math.atan2(
            self.intermediate_cog_y - self.human_center_y,
            self.intermediate_cog_x - self.human_center_x
        )

        final_angle = math.atan2(
            self.target_robot_head_pose.pose.position.y - self.human_center_y,
            self.target_robot_head_pose.pose.position.x - self.human_center_x
        )

        # Calculate angular difference and normalize to [-pi, pi]
        angle_diff = final_angle - intermediate_angle
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi

        rospy.loginfo(
            f"Arc motion: from {math.degrees(intermediate_angle):.1f} deg to {math.degrees(final_angle):.1f} deg (sweep: {math.degrees(angle_diff):.1f} deg)")

        # Generate waypoints along the arc
        self.arc_waypoints = []
        for i in range(1, self.num_arc_waypoints + 1):
            # Interpolate angle
            t = float(i) / self.num_arc_waypoints
            current_angle = intermediate_angle + angle_diff * t

            # Calculate head position on the circle
            head_x = self.human_center_x + self.safe_distance * math.cos(current_angle)
            head_y = self.human_center_y + self.safe_distance * math.sin(current_angle)
            head_z = self.target_robot_head_pose.pose.position.z

            # Head orientation: pointing from H to head position
            # Since robot x-axis points from head to body, yaw points from H to head position
            head_yaw = current_angle

            # Create transformation matrix for this waypoint head pose
            waypoint_head_matrix = tf.transformations.concatenate_matrices(
                tf.transformations.translation_matrix([head_x, head_y, head_z]),
                tf.transformations.quaternion_matrix(
                    tf.transformations.quaternion_from_euler(0, 0, head_yaw)
                )
            )

            # Calculate CoG pose for this waypoint
            waypoint_cog_matrix = tf.transformations.concatenate_matrices(
                waypoint_head_matrix, root_to_cog_matrix
            )

            # Extract CoG position and orientation
            waypoint_cog_pos = tf.transformations.translation_from_matrix(waypoint_cog_matrix)
            waypoint_cog_quat = tf.transformations.quaternion_from_matrix(waypoint_cog_matrix)
            _, _, waypoint_cog_yaw = tf.transformations.euler_from_quaternion(waypoint_cog_quat)

            # Store waypoint as dict
            waypoint = {
                'cog_x': waypoint_cog_pos[0],
                'cog_y': waypoint_cog_pos[1],
                'cog_z': waypoint_cog_pos[2],
                'cog_yaw': waypoint_cog_yaw,
                'head_x': head_x,
                'head_y': head_y,
                'head_yaw': head_yaw
            }
            self.arc_waypoints.append(waypoint)

        rospy.loginfo(f"Generated {len(self.arc_waypoints)} waypoints for arc motion")

    def publish_intermediate_pose_marker(self, x, y, z, yaw):
        """Publish a marker to visualize the intermediate head pose in RViz"""
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "intermediate_pose"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        # Position
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z

        # Orientation
        quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
        marker.pose.orientation.x = quat[0]
        marker.pose.orientation.y = quat[1]
        marker.pose.orientation.z = quat[2]
        marker.pose.orientation.w = quat[3]

        # Scale (arrow dimensions)
        marker.scale.x = 0.3  # Arrow length
        marker.scale.y = 0.05  # Arrow width
        marker.scale.z = 0.05  # Arrow height

        # Color (green for intermediate pose)
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.lifetime = rospy.Duration(0)  # Permanent until replaced

        self.marker_pub.publish(marker)
        rospy.loginfo("Published intermediate pose marker to RViz")

    def publish_safety_circle_marker(self, center_x, center_y, z, radius):
        """Publish a marker to visualize the safety circle in RViz"""
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "safety_circle"
        marker.id = 1
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        # Position (set to origin for LINE_STRIP, points define actual positions)
        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = 0
        marker.pose.orientation.w = 1.0

        # Line width
        marker.scale.x = 0.02

        # Color (red for safety circle)
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.8

        # Generate circle points
        num_points = 50
        for i in range(num_points + 1):  # +1 to close the circle
            angle = 2 * math.pi * i / num_points
            point = geometry_msgs.msg.Point()
            point.x = center_x + radius * math.cos(angle)
            point.y = center_y + radius * math.sin(angle)
            point.z = z
            marker.points.append(point)

        marker.lifetime = rospy.Duration(0)  # Permanent until replaced

        self.marker_pub.publish(marker)
        rospy.loginfo(f"Published safety circle marker at ({center_x:.3f}, {center_y:.3f}) with radius {radius:.3f}m")

    def publish_human_center_marker(self, center_x, center_y, z):
        """Publish a marker to visualize the human center (point H) in RViz"""
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "human_center"
        marker.id = 2
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        # Position
        marker.pose.position.x = center_x
        marker.pose.position.y = center_y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0

        # Scale (sphere diameter)
        marker.scale.x = 0.15
        marker.scale.y = 0.15
        marker.scale.z = 0.15

        # Color (blue for human center)
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        marker.lifetime = rospy.Duration(0)  # Permanent until replaced

        self.marker_pub.publish(marker)
        rospy.loginfo(f"Published human center marker at ({center_x:.3f}, {center_y:.3f})")

    def publish_final_head_pose_marker(self, x, y, z, yaw):
        """Publish a marker to visualize the final robot head pose in RViz"""
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "final_head_pose"
        marker.id = 3
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        # Position
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z

        # Orientation
        quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
        marker.pose.orientation.x = quat[0]
        marker.pose.orientation.y = quat[1]
        marker.pose.orientation.z = quat[2]
        marker.pose.orientation.w = quat[3]

        # Scale (arrow dimensions)
        marker.scale.x = 0.3  # Arrow length
        marker.scale.y = 0.05  # Arrow width
        marker.scale.z = 0.05  # Arrow height

        # Color (yellow for final target pose)
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.lifetime = rospy.Duration(0)  # Permanent until replaced

        self.marker_pub.publish(marker)
        rospy.loginfo("Published final head pose marker to RViz")

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
        if self.current_cog_pose is None:
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

        # Check based on current navigation stage
        if self.navigation_stage == 'intermediate':
            if self.intermediate_cog_x is None:
                return False

            # Calculate position error for intermediate pose
            dx = current_cog_x - self.intermediate_cog_x
            dy = current_cog_y - self.intermediate_cog_y
            dz = current_cog_z - self.intermediate_cog_z
            distance = math.sqrt(dx*dx + dy*dy + dz*dz)

            # Normalize yaw error to [-pi, pi]
            yaw_error = self.intermediate_cog_yaw - current_yaw
            while yaw_error > math.pi:
                yaw_error -= 2 * math.pi
            while yaw_error < -math.pi:
                yaw_error += 2 * math.pi

            if distance < self.nav_position_tolerance and abs(yaw_error) < self.nav_yaw_tolerance:
                rospy.loginfo("Intermediate position reached! Starting arc motion...")
                self.navigation_stage = 'arc_motion'
                self.current_waypoint_index = 0

                # Publish navigation command for first arc waypoint
                if len(self.arc_waypoints) > 0:
                    waypoint = self.arc_waypoints[self.current_waypoint_index]
                    nav_msg = FlightNav()
                    nav_msg.control_frame = FlightNav.WORLD_FRAME
                    nav_msg.target = FlightNav.COG
                    nav_msg.pos_xy_nav_mode = FlightNav.POS_MODE
                    nav_msg.pos_z_nav_mode = FlightNav.POS_MODE
                    nav_msg.yaw_nav_mode = FlightNav.POS_MODE
                    nav_msg.target_pos_x = waypoint['cog_x']
                    nav_msg.target_pos_y = waypoint['cog_y']
                    nav_msg.target_pos_z = waypoint['cog_z']
                    nav_msg.target_yaw = waypoint['cog_yaw']
                    self.uav_nav_pub.publish(nav_msg)
                    rospy.loginfo(f"Stage 2: Arc motion - navigating to waypoint {self.current_waypoint_index + 1}/{len(self.arc_waypoints)}")
                return False  # Not finished yet, moving to arc motion
            return False

        elif self.navigation_stage == 'arc_motion':
            if len(self.arc_waypoints) == 0:
                return False

            # Get current waypoint
            waypoint = self.arc_waypoints[self.current_waypoint_index]

            # Calculate position error for current waypoint
            dx = current_cog_x - waypoint['cog_x']
            dy = current_cog_y - waypoint['cog_y']
            dz = current_cog_z - waypoint['cog_z']
            distance = math.sqrt(dx*dx + dy*dy + dz*dz)

            # Normalize yaw error to [-pi, pi]
            yaw_error = waypoint['cog_yaw'] - current_yaw
            while yaw_error > math.pi:
                yaw_error -= 2 * math.pi
            while yaw_error < -math.pi:
                yaw_error += 2 * math.pi

            if distance < self.nav_position_tolerance and abs(yaw_error) < self.nav_yaw_tolerance:
                # Current waypoint reached
                self.current_waypoint_index += 1

                if self.current_waypoint_index < len(self.arc_waypoints):
                    # Move to next waypoint
                    waypoint = self.arc_waypoints[self.current_waypoint_index]
                    nav_msg = FlightNav()
                    nav_msg.control_frame = FlightNav.WORLD_FRAME
                    nav_msg.target = FlightNav.COG
                    nav_msg.pos_xy_nav_mode = FlightNav.POS_MODE
                    nav_msg.pos_z_nav_mode = FlightNav.POS_MODE
                    nav_msg.yaw_nav_mode = FlightNav.POS_MODE
                    nav_msg.target_pos_x = waypoint['cog_x']
                    nav_msg.target_pos_y = waypoint['cog_y']
                    nav_msg.target_pos_z = waypoint['cog_z']
                    nav_msg.target_yaw = waypoint['cog_yaw']
                    self.uav_nav_pub.publish(nav_msg)
                    rospy.loginfo(f"Arc motion - navigating to waypoint {self.current_waypoint_index + 1}/{len(self.arc_waypoints)}")
                    return False
                else:
                    # All arc waypoints completed, move to final position
                    rospy.loginfo("Arc motion complete! Moving to final handover position...")
                    self.navigation_stage = 'final'

                    # Publish navigation command for final pose
                    nav_msg = FlightNav()
                    nav_msg.control_frame = FlightNav.WORLD_FRAME
                    nav_msg.target = FlightNav.COG
                    nav_msg.pos_xy_nav_mode = FlightNav.POS_MODE
                    nav_msg.pos_z_nav_mode = FlightNav.POS_MODE
                    nav_msg.yaw_nav_mode = FlightNav.POS_MODE
                    nav_msg.target_pos_x = self.target_cog_x
                    nav_msg.target_pos_y = self.target_cog_y
                    nav_msg.target_pos_z = self.target_cog_z
                    nav_msg.target_yaw = self.target_cog_yaw
                    self.uav_nav_pub.publish(nav_msg)
                    rospy.loginfo(f"Stage 3: Navigating to final handover position...")
                    return False
            return False

        elif self.navigation_stage == 'final':
            if self.target_cog_x is None:
                return False

            # Calculate position error for final pose
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

        return False

    def run(self):
        """Main loop"""
        rospy.spin()


if __name__ == '__main__':
    try:
        nav = DragonSimpleNav()
        nav.run()
    except rospy.ROSInterruptException:
        pass
