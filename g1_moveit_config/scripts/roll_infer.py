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
    def __init__(self, model_path, feat, lookahead=0, n_action_steps=0, chain=False, gate=False):
        super().__init__('g1_act_infer', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.names = feat.JOINT_NAMES
        self.pub = self.create_publisher(JointTrajectory, '/g1_controller/joint_trajectory', 10)
        self.create_subscription(JointState, feat.STATE_TOPIC, self.on_joints, 10)
        self.policy = ACTPolicy.from_pretrained(model_path)
        self.policy.eval()
        self.policy.to(torch.device('cpu'))
        if n_action_steps:   # 追加: 予測した指令の列を何個ずつ流すか（学習時の設定を上書き）
            self.policy.config.n_action_steps = n_action_steps
        self.policy.reset()
        self.state = None
        self.lookahead = lookahead
        self.chain = chain
        self.prev = None
        self.feat = feat
        self.env = None
        if hasattr(feat, 'ENV_TOPIC'):   # 追加: 骨盤の回転センサを environment_state に使う（特徴量の第 2 版）
            from nav_msgs.msg import Odometry
            self.create_subscription(Odometry, feat.ENV_TOPIC, self.on_odom, 10)
        self.enabled = not gate
        if gate:   # 追加: /roll/enable（Bool）が True の間だけ指令を出す（モデルを読み込んだまま何回も動かす）
            from std_msgs.msg import Bool
            self.create_subscription(Bool, '/roll/enable', self.on_enable, 10)
            self.ready_pub = self.create_publisher(Bool, '/roll/ready', 10)
            self.create_timer(0.5, lambda: self.ready_pub.publish(Bool(data=True)))
        self.create_timer(0.02, self.tick)  # 20 Hz（シミュレータの時間）
        self.get_logger().info('ready')

    def on_enable(self, msg):
        if msg.data and not self.enabled:   # 動かし始めるたびに、方策の指令の列と前の目標を捨てる
            self.policy.reset()
            self.prev = None
        self.enabled = msg.data

    def on_joints(self, msg):
        m = dict(zip(msg.name, msg.position))
        if all(n in m for n in self.names):
            self.state = np.array([m[n] for n in self.names], dtype=np.float32)

    def on_odom(self, m):
        o, w, p = m.pose.pose.orientation, m.twist.twist.angular, m.pose.pose.position
        self.env = np.array(self.feat.env_from_odom(o.x, o.y, o.z, o.w, w.x, w.y, w.z, p.z), dtype=np.float32)

    def tick(self):
        if not self.enabled or self.state is None or (hasattr(self.feat, 'ENV_TOPIC') and self.env is None):
            return
        s = torch.from_numpy(self.state).unsqueeze(0)
        e = s if self.env is None else torch.from_numpy(self.env).unsqueeze(0)
        obs = {'observation.state': s, 'observation.environment_state': e}
        with torch.no_grad():
            if self.lookahead:   # 追加: 予測した指令の列（0.1 秒刻み）から k 個先を使う（関節の追従遅れで動きが鈍るのを補う）
                b = self.policy.normalize_inputs(obs)
                chunk = self.policy.unnormalize_outputs({'action': self.policy.model(b)[0]})['action']
                action = chunk[0, self.lookahead].cpu().numpy().flatten()
            else:
                action = self.policy.select_action(obs).cpu().numpy().flatten()
        msg = JointTrajectory()
        msg.joint_names = self.names
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in action]
        point.time_from_start = Duration(sec=0, nanosec=20_000_000)
        msg.points = [point]
        if self.chain and self.prev is not None:   # 追加: 前に出した目標（0 秒）→ 新しい目標（0.02 秒）の 2 点で送る
            p0 = JointTrajectoryPoint()            # 1 点だけだと、コントローラが遅れている関節の今の角度から補間し直し、
            p0.positions = self.prev               # 目標が今の角度に引き戻されて動きが鈍る（お手本は区間ごとの軌道で送っていた）
            p0.time_from_start = Duration(sec=0, nanosec=0)
            msg.points = [p0, point]
        self.prev = list(point.positions)
        self.pub.publish(msg)


def main():
    ap = argparse.ArgumentParser(description='ACTPolicy inference node for Unitree G1 (ROS 2)')
    ap.add_argument('--model-path', required=True)
    ap.add_argument('--config', required=True, help='特徴量の定義ファイル（JOINT_NAMES など）')
    ap.add_argument('--lookahead', type=int, default=0, help='追加: 指令の列の何個先（0.1 秒刻み）を使うか')
    ap.add_argument('--n-action-steps', type=int, default=0, help='追加: 指令の列を何個ずつ流すか（0 なら学習時の設定）')
    ap.add_argument('--chain', action='store_true', help='追加: 前の目標→新しい目標の 2 点で送る')
    ap.add_argument('--gate', action='store_true', help='追加: /roll/enable が True の間だけ動かす')
    args = ap.parse_args()
    rclpy.init()
    node = G1Infer(args.model_path, load_py(args.config), args.lookahead, args.n_action_steps, args.chain,
                   args.gate)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
