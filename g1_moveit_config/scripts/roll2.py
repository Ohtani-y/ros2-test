#!/usr/bin/env python3
"""学習済みモデル（g1_backroll_act_v2）で後転を 2 回続けて行う、シンプル版。

前提: 次の 2 つが起動済みであること
  ros2 launch g1_gazebo g1_lying.launch.py gui:=true
  ros2 launch g1_moveit_config moveit.launch.py
使い方:
  cd /workspace/g1_moveit_config/scripts && python3 roll2.py
動画は ~/deploy/g1_backroll_x2/roll_x2.mp4 に保存される。
"""
import os
import signal
import subprocess
import sys
import time

import rclpy
from builtin_interfaces.msg import Duration
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import roll_trial as R  # noqa: E402  関節名 JOINTS・回転角 Roll・背骨の向き spine
from deploy_roll import raise_gazebo  # noqa: E402  Gazebo window to front

MODEL = '/workspace/checkpoints/g1_backroll_act_v2'
CFG = '/workspace/pai_training_g1/configs/g1_backroll_features_v2.py'
TIMES = 2                                   # 回数
OUT = os.path.expanduser('~/deploy/g1_backroll_x2')

rclpy.init()
node = rclpy.create_node('roll2')
pub = node.create_publisher(JointTrajectory, '/g1_controller/joint_trajectory', 10)
enable = node.create_publisher(Bool, '/roll/enable', 10)
st = {'t': None, 'q': None, 'ready': False}


def on_odom(m):                             # 回転センサ（骨盤の姿勢）とシミュレータ時刻
    st['t'] = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
    o = m.pose.pose.orientation
    st['q'] = (o.x, o.y, o.z, o.w)


node.create_subscription(Odometry, '/model/g1/odometry', on_odom, 50)
node.create_subscription(Bool, '/roll/ready', lambda m: st.update(ready=True), 10)


def wait(sec, until=None):                  # シミュレータの時間で sec 秒待つ（until が真なら途中で抜ける）
    while st['t'] is None:
        rclpy.spin_once(node, timeout_sec=0.1)
    end = st['t'] + sec
    while st['t'] < end:
        rclpy.spin_once(node, timeout_sec=0.02)
        if until and until():
            return True
    return False


def reset():                                # 全関節 0 → 骨盤を仰向けに置き直す
    msg = JointTrajectory(joint_names=R.JOINTS)
    msg.points.append(JointTrajectoryPoint(positions=[0.0] * len(R.JOINTS), time_from_start=Duration(sec=1)))
    pub.publish(msg)
    wait(1.5)
    subprocess.run(['ign', 'service', '-s', '/world/ground/set_pose',
                    '--reqtype', 'ignition.msgs.Pose', '--reptype', 'ignition.msgs.Boolean', '--timeout', '3000',
                    '--req', 'name: "g1", position: {x: 0, y: 0, z: 0.12}, '
                             'orientation: {x: 0, y: -0.70710678, z: 0, w: 0.70710678}'],
                   capture_output=True, timeout=10)
    wait(2.5)


def finished(roll):                         # 330° 以上回って、仰向け（腹が上）に戻った
    q = st['q']
    ang = roll(q)
    return (ang is not None and ang >= 330 and abs(R.spine(*q)[2]) < 0.3
            and 2 * (q[0] * q[2] - q[1] * q[3]) > 0.8)


# 1) 推論ノードを起動（--gate: /roll/enable が True の間だけ方策が指令を出す）
env = dict(os.environ, PATH=os.path.expanduser('~/.pixi/bin') + ':' + os.environ['PATH'])
infer = subprocess.Popen(['pixi', 'run', 'python', os.path.join(HERE, 'roll_infer.py'),
                          '--model-path', MODEL, '--config', CFG,
                          '--chain', '--n-action-steps', '10', '--gate'],
                         cwd=os.path.expanduser('~/pai_training'), env=env, start_new_session=True)

# 2) リセットし、モデルの読み込みが終わるのを待つ
reset()
limit = time.time() + 180
while not st['ready'] and time.time() < limit:
    rclpy.spin_once(node, timeout_sec=0.1)

# 3) 画面の録画を始める（画面全体）
os.makedirs(OUT, exist_ok=True)
try:  # move the mouse pointer to the screen center (no taskbar hover preview in the video)
    from Xlib import display as _xd
    _d = _xd.Display(); _s = _d.screen()
    _s.root.warp_pointer(_s.width_in_pixels // 2, _s.height_in_pixels // 2); _d.sync()
except Exception:
    pass
raise_gazebo()
ff = subprocess.Popen(['ffmpeg', '-y', '-loglevel', 'error', '-f', 'x11grab', '-draw_mouse', '0', '-framerate', '15',
                       '-i', os.environ.get('DISPLAY', ':0'), '-vf', 'scale=960:-2',
                       '-c:v', 'libx264', '-preset', 'ultrafast', '-pix_fmt', 'yuv420p',
                       os.path.join(OUT, 'roll_x2.mp4')], stdin=subprocess.PIPE)

# 4) 2 回続けて後転（2 回目はリセットせず、1 回目の終わりの姿勢から始める）
for i in range(1, TIMES + 1):
    roll = R.Roll(st['q'])                  # この回の始まりの向きを 0° とする
    enable.publish(Bool(data=True))
    ok = wait(8.0, until=lambda: finished(roll))
    enable.publish(Bool(data=False))
    wait(1.0)                               # 姿勢が落ち着くのを待つ
    print(f'roll {i}: {"SUCCESS" if ok else "fail"}  {roll(st["q"]) or 0:.1f} deg', flush=True)
    if not ok:
        break

# 5) 後片付け
wait(1.5)
ff.communicate(b'q', timeout=15)            # q を送って録画を正しく閉じる
os.killpg(infer.pid, signal.SIGINT)
rclpy.shutdown()
