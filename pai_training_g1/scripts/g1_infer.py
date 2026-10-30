#!/usr/bin/env python3
"""
G1 用の推論ノード。講義の scripts/infer.py と同じ作り（/joint_states を読み、方策 ACT の出力を
20 Hz で関節指令に送る）で、次の 2 点を変えた。
  - 関節を 29 個にし、特徴量の定義ファイル（JOINT_NAMES）の順に並べる
  - 時計をシミュレータの時間にする（この PC は実時間比が約 0.5 のため。データも同じ時計で記録した）

使い方（ROS を読み込んだシェルで、~/pai_training の中）:
  pixi run python /workspace/pai_training_g1/scripts/g1_infer.py --model-path <学習結果> \
      --config /workspace/pai_training_g1/configs/g1_lesson2_fours_features.py
"""

import argparse
import importlib.util

import numpy as np
import rclpy
import torch
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from lerobot.common.policies.act.modeling_act import ACTPolicy


def load_py(path):
    spec = importlib.util.spec_from_file_location('g1_features', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class G1Infer(Node):
    def __init__(self, model_path, feat):
        super().__init__('g1_act_infer', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.names = feat.JOINT_NAMES
        self.pub = self.create_publisher(JointTrajectory, feat.ACTION_TOPIC, 10)
        self.create_subscription(JointState, feat.STATE_TOPIC, self.on_joints, 10)
        self.policy = ACTPolicy.from_pretrained(model_path)
        self.policy.eval()
        self.policy.to(torch.device('cpu'))
        self.policy.reset()
        self.state = None
        self.create_timer(0.05, self.tick)  # 20 Hz（シミュレータの時間）
        self.get_logger().info('ready')

    def on_joints(self, msg):
        m = dict(zip(msg.name, msg.position))
        if all(n in m for n in self.names):
            self.state = np.array([m[n] for n in self.names], dtype=np.float32)

    def tick(self):
        if self.state is None:
            return
        s = torch.from_numpy(self.state).unsqueeze(0)
        with torch.no_grad():
            action = self.policy.select_action(
                {'observation.state': s, 'observation.environment_state': s}).cpu().numpy().flatten()
        msg = JointTrajectory()
        msg.joint_names = self.names
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in action]
        point.time_from_start = Duration(sec=0, nanosec=100_000_000)
        msg.points = [point]
        self.pub.publish(msg)


def main():
    ap = argparse.ArgumentParser(description='ACTPolicy inference node for Unitree G1 (ROS 2)')
    ap.add_argument('--model-path', required=True)
    ap.add_argument('--config', required=True, help='特徴量の定義ファイル（JOINT_NAMES など）')
    args = ap.parse_args()
    rclpy.init()
    node = G1Infer(args.model_path, load_py(args.config))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
