#!/usr/bin/env python3
"""学習した方策で後転を N 回続けて行う（既定 3 回）。動画は全体を 1 本に撮る。
推論ノード（roll_infer.py --gate）は最初に 1 回だけ起動し、/roll/enable で動かす・止めるを切り替える。
1 回分 = 動かす → 1 回転して仰向けに戻ったら止める（deploy_roll.py の --stop-on-goal と同じ判定）→ settle-s 待つ。
最初の 1 回の前だけリセットし、2 回目以降は前の回の終わりの姿勢からそのまま始める。
  python3 roll3.py [--times 3] [--settle-s 1.0] [--out ~/deploy/g1_backroll_x3]"""
import argparse
import json
import os
import signal
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import roll_trial as R  # noqa: E402
from deploy_roll import raise_gazebo  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
MODEL = '/workspace/checkpoints/g1_backroll_act_v2'
CFG = '/workspace/pai_training_g1/configs/g1_backroll_features_v2.py'


def main():
    import rclpy
    from builtin_interfaces.msg import Duration
    from nav_msgs.msg import Odometry
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    ap = argparse.ArgumentParser()
    ap.add_argument('--times', type=int, default=3, help='何回続けて後転するか')
    ap.add_argument('--timeout-s', type=float, default=8.0, help='1 回の上限（シミュレータの時間）')
    ap.add_argument('--settle-s', type=float, default=1.0, help='止めてから次の回までの待ち（シミュレータの時間）')
    ap.add_argument('--out', default=os.path.expanduser('~/deploy/g1_backroll_x3'))
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)

    rclpy.init()
    node = rclpy.create_node('roll3')
    pub = node.create_publisher(JointTrajectory, '/g1_controller/joint_trajectory', 10)
    st = {'now': None, 'q': None}

    def on_odom(m):
        st['now'] = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        o = m.pose.pose.orientation
        st['q'] = (o.x, o.y, o.z, o.w)
    node.create_subscription(Odometry, '/model/g1/odometry', on_odom, 50)
    while st['now'] is None:
        rclpy.spin_once(node, timeout_sec=0.1)

    def wait_sim(sec, until=None):
        t_end = st['now'] + sec
        while st['now'] < t_end:
            rclpy.spin_once(node, timeout_sec=0.02)
            if until and until():
                return True
        return False

    def reset():   # 仰向け・全関節 0（deploy_roll.py と同じ）
        msg = JointTrajectory()
        msg.joint_names = R.JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = [0.0] * len(R.JOINTS)
        pt.time_from_start = Duration(sec=1)
        msg.points.append(pt)
        pub.publish(msg)
        wait_sim(1.5)
        subprocess.run(['ign', 'service', '-s', '/world/ground/set_pose', '--reqtype', 'ignition.msgs.Pose',
                        '--reptype', 'ignition.msgs.Boolean', '--timeout', '3000', '--req',
                        'name: "g1", position: {x: 0, y: 0, z: 0.12}, '
                        'orientation: {x: 0, y: -0.70710678, z: 0, w: 0.70710678}'],
                       capture_output=True, timeout=10)
        wait_sim(2.5)

    def done(roll):   # 1 回転（330° 以上）して仰向けに戻った
        ang = roll(st['q'])
        q = st['q']
        return (ang is not None and ang >= 330 and abs(R.spine(*q)[2]) < 0.3
                and 2 * (q[0] * q[2] - q[1] * q[3]) > 0.8)

    env = dict(os.environ, PATH=os.path.expanduser('~/.pixi/bin') + ':' + os.environ['PATH'])

    # 推論ノードは最初に 1 回だけ起動する（モデルの読み込みをリセットと並行して済ませる）。
    # 動かす・止めるは /roll/enable で切り替えるので、回と回の間に読み込みの待ちが入らない。
    from std_msgs.msg import Bool
    en_pub = node.create_publisher(Bool, '/roll/enable', 10)
    st['ready'] = False
    node.create_subscription(Bool, '/roll/ready', lambda m: st.update(ready=True), 10)
    proc = subprocess.Popen(['pixi', 'run', 'python', os.path.join(HERE, 'roll_infer.py'),
                             '--model-path', MODEL, '--config', CFG, '--chain', '--n-action-steps', '10', '--gate'],
                            cwd=os.path.expanduser('~/pai_training'), env=env,
                            stdout=open(os.path.join(a.out, 'infer.log'), 'w'),
                            stderr=subprocess.STDOUT, start_new_session=True)

    def roll_once(i):
        """方策を動かし、1 回転して仰向けに戻ったら止める。結果を返す。"""
        roll = R.Roll(st['q'])          # この回の始まりの向きを 0° とする
        t0 = st['now']
        en_pub.publish(Bool(data=True))
        ok = wait_sim(a.timeout_s, until=lambda: done(roll))
        en_pub.publish(Bool(data=False))
        wait_sim(a.settle_s)             # 止めたあと、姿勢が落ち着くのを少し待つ
        res = dict(roll=i, success=ok, angle=round(roll(st['q']) or 0, 1), sim_s=round(st['now'] - t0, 2))
        print(f'roll {i}: {"SUCCESS" if ok else "fail"}  angle {res["angle"]}°  ({res["sim_s"]} s)', flush=True)
        return res

    reset()
    t_w = time.time() + 180
    while not st['ready'] and time.time() < t_w:   # 読み込みが終わるのを待つ（リセット中に進んでいる）
        rclpy.spin_once(node, timeout_sec=0.1)
    raise_gazebo()
    ff = subprocess.Popen(['ffmpeg', '-y', '-loglevel', 'error', '-f', 'x11grab', '-draw_mouse', '0', '-framerate', '15',
                           '-video_size', '1918x988', '-i', os.environ.get('DISPLAY', ':0') + '+1,62',
                           '-vf', 'scale=960:-2', '-c:v', 'libx264', '-preset', 'ultrafast', '-crf', '30',
                           '-pix_fmt', 'yuv420p', os.path.join(a.out, 'roll_x3.mp4')],
                          stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    results = []
    for i in range(1, a.times + 1):     # ← ここで N 回呼び出す
        r = roll_once(i)
        results.append(r)
        if not r['success']:
            break

    wait_sim(1.5)                        # 最後の姿勢を少し映してから録画を止める
    ff.communicate(b'q', timeout=15)
    os.killpg(proc.pid, signal.SIGINT)
    try:
        proc.wait(timeout=20)
    except Exception:
        os.killpg(proc.pid, signal.SIGKILL)
    with open(os.path.join(a.out, 'results.json'), 'w') as f:
        json.dump(dict(time=time.strftime('%F %T'), model=MODEL, results=results), f, indent=1)
    print(f'success {sum(r["success"] for r in results)}/{a.times}', flush=True)


if __name__ == '__main__':
    main()
