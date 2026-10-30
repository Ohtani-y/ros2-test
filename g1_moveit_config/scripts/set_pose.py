#!/usr/bin/env python3
"""Send a named whole-body pose to g1_controller.
  python3 set_pose.py forward_roll [seconds]
  python3 set_pose.py zero [seconds]
Joints not listed in a pose go to 0 rad."""
import os
import sys
import yaml
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from ament_index_python.packages import get_package_share_directory

POSES = {
    'zero': {},
    # mid forward roll (photo): weight on the upper back and shoulders, hips high,
    # legs almost straight and swung over the face, elbows forward, hands beside the ears palms up (protect the head)
    'forward_roll': {
        'left_hip_pitch_joint': -2.5, 'right_hip_pitch_joint': -2.5,
        'left_knee_joint': 0.4, 'right_knee_joint': 0.4,
        'left_shoulder_pitch_joint': -1.7, 'right_shoulder_pitch_joint': -1.7,
        'left_shoulder_roll_joint': 0.45, 'right_shoulder_roll_joint': -0.45,
        'left_elbow_joint': 2.0, 'right_elbow_joint': 2.0,
    },
    # mid back roll: back on the floor, hips flexed so the legs swing over the head,
    # knees tucked, arms overhead with elbows bent (hands next to the ears)
    'back_roll': {
        'left_hip_pitch_joint': -2.3, 'right_hip_pitch_joint': -2.3,
        'left_knee_joint': 1.8, 'right_knee_joint': 1.8,
        'left_shoulder_pitch_joint': -2.6, 'right_shoulder_pitch_joint': -2.6,
        'left_shoulder_roll_joint': 0.3, 'right_shoulder_roll_joint': -0.3,
        'left_elbow_joint': 1.5, 'right_elbow_joint': 1.5,
    },
    # STEP1 (forward roll): saved from the RViz goal state on 2026-10-11 (read in whole degrees)
    'STEP1': {
        'left_hip_pitch_joint': -2.5307,  # -145 deg
        'left_hip_roll_joint': 0.0,  # 0 deg
        'left_hip_yaw_joint': 0.0,  # 0 deg
        'left_knee_joint': 0.0,  # 0 deg
        'left_ankle_pitch_joint': 0.0,  # 0 deg
        'left_ankle_roll_joint': 0.0,  # 0 deg
        'right_hip_pitch_joint': -2.5307,  # -145 deg
        'right_hip_roll_joint': 0.0,  # 0 deg
        'right_hip_yaw_joint': 0.0,  # 0 deg
        'right_knee_joint': 0.0,  # 0 deg
        'right_ankle_pitch_joint': 0.0,  # 0 deg
        'right_ankle_roll_joint': 0.0,  # 0 deg
        'waist_yaw_joint': 0.0,  # 0 deg
        'waist_roll_joint': 0.0,  # 0 deg
        'waist_pitch_joint': 0.0524,  # 3 deg
        'left_shoulder_pitch_joint': -1.9897,  # -114 deg
        'left_shoulder_roll_joint': 0.6807,  # 39 deg
        'left_shoulder_yaw_joint': 0.4014,  # 23 deg
        'left_elbow_joint': -0.6981,  # -40 deg
        'left_wrist_roll_joint': 1.3439,  # 77 deg
        'left_wrist_pitch_joint': -0.1222,  # -7 deg
        'left_wrist_yaw_joint': 1.2392,  # 71 deg
        'right_shoulder_pitch_joint': -2.1118,  # -121 deg
        'right_shoulder_roll_joint': -0.8029,  # -46 deg
        'right_shoulder_yaw_joint': -0.192,  # -11 deg
        'right_elbow_joint': -0.733,  # -42 deg
        'right_wrist_roll_joint': -1.4137,  # -81 deg
        'right_wrist_pitch_joint': 0.0873,  # 5 deg
        'right_wrist_yaw_joint': -1.2217,  # -70 deg
    },
}


def main():
    name = sys.argv[1] if len(sys.argv) > 1 else 'forward_roll'
    sec = float(sys.argv[2]) if len(sys.argv) > 2 else 3.0
    yml = os.path.join(get_package_share_directory('g1_gazebo'), 'config', 'g1_controllers.yaml')
    joints = yaml.safe_load(open(yml))['g1_controller']['ros__parameters']['joints']
    pose = POSES[name]
    rclpy.init()
    node = Node('set_pose')
    pub = node.create_publisher(JointTrajectory, '/g1_controller/joint_trajectory', 10)
    while pub.get_subscription_count() == 0:
        rclpy.spin_once(node, timeout_sec=0.1)
    msg = JointTrajectory()
    msg.joint_names = joints
    pt = JointTrajectoryPoint()
    pt.positions = [float(pose.get(j, 0.0)) for j in joints]
    pt.time_from_start = Duration(sec=int(sec), nanosec=int((sec % 1) * 1e9))
    msg.points = [pt]
    pub.publish(msg)
    rclpy.spin_once(node, timeout_sec=0.5)
    print(f'sent {name} ({sec}s)')


if __name__ == '__main__':
    main()
