#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import numpy as np
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_msgs.msg import Empty
from tf.transformations import quaternion_from_euler, quaternion_matrix, quaternion_multiply

class HandFollower:
    def __init__(self):
        rospy.init_node('hand_follower')

        self.hand_topic = rospy.get_param("~hand_topic", "/hand/pose")
        self.eye_topic = rospy.get_param("~eye_topic", "/eye/pose")

        self.eye_offset = rospy.get_param("~eye_offset", 0.4)

        self.hand_pub = rospy.Publisher(self.hand_topic, PoseStamped, queue_size = 1)
        self.eye_pub = rospy.Publisher(self.eye_topic, PoseStamped, queue_size = 1)

        rospy.loginfo("Eye-Chasing-Hand Started.")
        rospy.loginfo(" hand_topic: %s", self.hand_topic)
        rospy.loginfo(" eye_topic: %s", self.eye_topic)
        rospy.loginfo(" eye_offset: %.3f [m]", self.eye_offset)

    def compute_eye_pose(self, hand_pose):
        """
        手のPoseStampedから、手の平法線上にある目のPoseStampedを計算する。
        - 手の平法線：手座標系の+Z軸を法線と仮定
        - 目の位置： hand_pos + offset * normal_world
        - 目の姿勢： 手の姿勢を180度回転させて、「手の平を見る」向きにする
        """

        #手の位置ベクトル
        hx = hand_pose.pose.position.x
        hy = hand_pose.pose.position.y
        hz = hand_pose.pose.position.z

        #手のクオータニオン
        qh = hand_pose.pose.orientation
        q_hand = np.array([qh.x, qh.y, qh.z qh.w])

        #手座標系の+Z軸(0,0,1)をワールド座標系に変換
        #quaternion_matrixで4x4の回転行列にして、その3x3部分を取り出す。
        R = quaternion_matrix(q_hand)[0:3, 0:3]
        normal_hand_frame = np.array([0.0, 0.0, 1.0])
        normal_world = R.dot(normal_hand_frame)
        normal_world = normal_world / np.linalg.norm(normal_world)

        #目の位置 = 手の位置 + offset * 法線
        eye_pos = hand_pos + self.eye_offset * normal_world

        #目の向き：
        #ここでは「手の平方向を向く」ように、手の姿勢を180度回転させておく。
        #回転軸は(0,1,0)　（手座標系のy軸）としているが、
        #　モデルによっては(1,0,0)や(0,0,1)のほうが自然な場合もあるので調整する。

        q_rot180 = quaternion_from_euler(0.0, math.pi, 0.0)
        #y軸周り180deg

        eye_pose = PoseStamped()
        eye_pose.header.frame_id = hand_pose.header.frame_id
        eye_pose.header.stamp = rospy.Time.now()

        eye_pose.pose.position.x = eye_pos[0]
        eye_pose.pose.position.y = eye_pos[1]
        eye_pose.pose.position.z = eye_pos[2]

        eye_pose.pose.orientation.x = q_eye[0]
        eye_pose.pose.orientation.y = q_eye[1]
        eye_pose.pose.orientation.z = q_eye[2]
        eye_pose.pose.orientation.w = q_eye[3]

        return eye_pose

    def run(self):
        """
        キーボードから手の位置姿勢を何度でも入力できるループ
        入力フォーマット：x y z roll_deg pitch_deg yaw_deg
        (終了したいときは、'q'を入力）
        """

        print("")
        print("=== Eye-Chasing-Hand ===")
        print("手の位置姿勢を入力すると、その手の平法線方向上に目を配置します。")
        print("入力フォーマット：x y z roll_deg pitch_deg yaw_deg")
        print("例：0.5 0.0 1.0 0 0 90")
        print("")

        while not rospy.is_shutdown():
            try:
                user_input = input("hand pose ->")
            except EOFError:
                break

            if user_input.strip().lower() == "q":
                rospy.loginfo("Quit requested.")
                break

            if not user_input.strip():
                continue

            try:
                vals = [float(v) for v in user_input.split()]
                if len(vals) != 6:
                    rospy.logwarn("6個の数値（x y z roll pitch yaw）を入力してください。")
                    continue
                x, y, z, roll_deg, pitch_deg, yaw_deg = vals
            except ValueError:
                rospy.logwarn("数値として解釈できませんでした。もう一度入力して下さい。")
                continue


            #角度をradに変換
            roll = math.radians(roll_deg)
            pitch = math.radians(pitch_deg)
            yaw = math.radians(yaw_deg)

            #手のクオータニオン
            q_hand = quaternion_from_euler(roll, pitch, yaw)

            #手のPoseStamedを作成
            hand_pose = PoseStamped
            hand_pose.header.stamp = rospy.Time.now()
            #frame_idは必要に応じて変更（"world"や"map"など）
            hand_pose.header.frame_id = "world"

            hand_pose.pose.position.x = x
            hand_pose.pose.position.y = y
            hand_pose.pose.position.z = z

            hand_pose.pose.orientation.x = q_hand[0]
            hand_pose.pose.orientation.y = q_hand[1]
            hand_pose.pose.orientation.z = q_hand[2]
            hand_pose.pose.orientation.w = q_hand[3]

            #目のPoseを計算
            eye_pose = self.compute_eye_pose(hand_pose)

            #Publish
            self.hand_pub.publish(hand_pose)
            self.eye_pub.publish(eye_pose)

            #コンソールに分かりやすく表示（確認しやすくするため）
            rospy.loginfo("Hand pose:")
            rospy.loginfo(" pos = (%.3f, %.3f, %.3f)", x, y, z)
            rospy.loginfo(" rpy = (%.1f, %.1f, %.1f) [deg]", roll_deg, pitch_deg, yaw_deg)

            rospy.loginfo("Eye pose:")
            rospy.loginfo(" pos = (%.3f, %.3f, %.3f)",
                          eye_pose.pose.position.x,
                          eye_pose.pose.position.y,
                          eye_pose.pose.position.z)

            #姿勢はクオータニオンのまま出しているが、必要ならrpyに戻してもよい
            qe = eye_pose.pose.orientation
            rospy.loginfo(" quat = (x=%.3f, y=%.3f, z=%.3f, w=%.3f)",
                          qe.x, qe.y, qe.z, qe.w)

            #すぐ次の入力を受け付けたいので、Rate.sleepだけ軽く回しておく
            rate.sleep()

if __name__ == "main__":
    node = HandFollower()
    node.run()
