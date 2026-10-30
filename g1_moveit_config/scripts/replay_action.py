#!/usr/bin/env python3
"""お手本の /roll/action を、方策と同じ送り方（シミュレータの時間で 0.02 秒ごとに 1 点、time_from_start 0.02 秒）で再生する。
送り方そのものに問題がないかを、方策なしで確かめるためのもの。
  pixi run python replay_action.py <bag のフォルダ>"""
import sys
import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from rclpy.parameter import Parameter
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ts_store = get_typestore(Stores.ROS2_HUMBLE)
seq, names = [], None
with Reader(sys.argv[1]) as r:
    for c, t, raw in r.messages(connections=[c for c in r.connections if c.topic == '/roll/action']):
        m = ts_store.deserialize_cdr(raw, c.msgtype)
        names = list(m.joint_names)
        seq.append(list(m.points[0].positions))
seq = np.array(seq)
k0 = np.where(np.abs(seq - seq[0]).max(1) > 0.02)[0][0]   # 動き出しから再生する
seq = seq[max(k0 - 2, 0):]
rclpy.init()
node = rclpy.create_node('replay_action', parameter_overrides=[Parameter('use_sim_time', value=True)])
pub = node.create_publisher(JointTrajectory, '/g1_controller/joint_trajectory', 10)
st = {'i': 0, 'prev': None}
CHAIN = '--chain' in sys.argv   # 前に出した目標（0 秒）→ 新しい目標（0.02 秒）の 2 点で送る（関節の遅れに引き戻されない）


def tick():
    q = seq[min(st['i'], len(seq) - 1)]
    st['i'] += 1
    msg = JointTrajectory()
    msg.joint_names = names
    pt = JointTrajectoryPoint()
    pt.positions = [float(v) for v in q]
    pt.time_from_start = Duration(sec=0, nanosec=20_000_000)
    msg.points = [pt]
    if CHAIN and st['prev'] is not None:
        p0 = JointTrajectoryPoint()
        p0.positions = st['prev']
        p0.time_from_start = Duration(sec=0, nanosec=0)
        msg.points = [p0, pt]
    st['prev'] = list(pt.positions)
    pub.publish(msg)


node.create_timer(0.02, tick)
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
