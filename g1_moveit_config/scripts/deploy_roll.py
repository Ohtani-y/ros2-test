#!/usr/bin/env python3
"""Deploy a trained backward-roll policy (LeRobot ACT) in Gazebo and score it.
Each trial: reset to lying on the back, start roll_infer.py (pixi env), let the policy run for --run-s
seconds of simulated time, measure the pelvis roll, stop the policy.
  python3 deploy_roll.py --model-path M [--trials 5] [--run-s 12] [--out ~/deploy/g1_backroll]"""
import argparse
import json
import math
import os
import signal
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import roll_trial as R  # noqa: E402

CFG = '/workspace/pai_training_g1/configs/g1_backroll_features.py'
INFER = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'roll_infer.py')


def raise_gazebo():
    """Bring the Gazebo GUI window to the front so the screen capture shows it (other windows may cover it)."""
    try:
        from Xlib import X, display, protocol
        d = display.Display()
        root = d.screen().root
        net_active = d.intern_atom('_NET_ACTIVE_WINDOW')

        def find(w):
            for c in w.query_tree().children:
                if (c.get_wm_class() or ('', ''))[0] == 'ign-gazebo-gui' and c.get_wm_name() == 'Gazebo':
                    return c
                r = find(c)
                if r:
                    return r
        w = find(root)
        if w:
            ev = protocol.event.ClientMessage(window=w, client_type=net_active, data=(32, [2, X.CurrentTime, 0, 0, 0]))
            root.send_event(ev, event_mask=X.SubstructureRedirectMask | X.SubstructureNotifyMask)
            w.raise_window()
            d.sync()
    except Exception as e:
        print('raise_gazebo:', e, flush=True)


def main():
    import rclpy
    from builtin_interfaces.msg import Duration
    from nav_msgs.msg import Odometry
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    ap = argparse.ArgumentParser()
    ap.add_argument('--model-path', default='')
    ap.add_argument('--replay', default='', help='bag folder: replay its /roll/action instead of running the policy')
    ap.add_argument('--trials', type=int, default=5)
    ap.add_argument('--run-s', type=float, default=12.0)
    ap.add_argument('--out', default=os.path.expanduser('~/deploy/g1_backroll'))
    ap.add_argument('--lookahead', type=int, default=0, help='use the k-th action of the predicted chunk')
    ap.add_argument('--stop-on-goal', action='store_true',
                    help='stop the policy once the roll reaches 330 deg and the robot lies on its back again')
    ap.add_argument('--chain', action='store_true', help='send (previous command at 0 s, new command at 0.02 s)')
    ap.add_argument('--n-action-steps', type=int, default=0, help='play this many actions of each predicted chunk')
    ap.add_argument('--config', default=CFG, help='features file given to roll_infer.py')
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node('deploy_roll')
    pub = node.create_publisher(JointTrajectory, '/g1_controller/joint_trajectory', 10)
    st = {'now': None, 'last': None, 'rec': False, 'samples': [], 'n_cmd': 0}

    def on_odom(m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        o, p = m.pose.pose.orientation, m.pose.pose.position
        st['now'] = t
        st['last'] = ((o.x, o.y, o.z, o.w), (p.x, p.y, p.z))
        if st['rec']:
            st['samples'].append((t, st['last'][0]))
    node.create_subscription(Odometry, '/model/g1/odometry', on_odom, 50)
    node.create_subscription(JointTrajectory, '/g1_controller/joint_trajectory',
                             lambda m: st.update(n_cmd=st['n_cmd'] + 1), 50)
    while st['now'] is None:
        rclpy.spin_once(node, timeout_sec=0.1)

    def wait_sim(sec):
        t_end = st['now'] + sec
        while st['now'] < t_end:
            rclpy.spin_once(node, timeout_sec=0.02)

    def reset():
        for _ in range(4):
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
            if abs(R.spine(*st['last'][0])[2]) < 0.1 and st['last'][1][2] < 0.2:
                return

    env = dict(os.environ, PATH=os.path.expanduser('~/.pixi/bin') + ':' + os.environ['PATH'])
    n_ok = 0
    for k in range(a.trials):
        reset()
        log = open(os.path.join(a.out, f'trial{k:02d}_infer.log'), 'w')
        n0 = st['n_cmd']
        cmd = ['pixi', 'run', 'python', INFER, '--model-path', a.model_path, '--config', a.config,
               '--lookahead', str(a.lookahead), '--n-action-steps', str(a.n_action_steps)] + \
              (['--chain'] if a.chain else [])
        if a.replay:   # check the command path without the policy: replay a demo's /roll/action the same way
            cmd = ['pixi', 'run', 'python', os.path.join(os.path.dirname(INFER), 'replay_action.py'), a.replay] + \
                  (['--chain'] if a.chain else [])
        proc = subprocess.Popen(cmd,
                                cwd=os.path.expanduser('~/pai_training'), env=env, stdout=log,
                                stderr=subprocess.STDOUT, start_new_session=True)
        t_w = time.time() + 180
        while st['n_cmd'] == n0 and time.time() < t_w and proc.poll() is None:   # wait for the first action
            rclpy.spin_once(node, timeout_sec=0.05)
        raise_gazebo()
        st['samples'] = []
        st['rec'] = True
        try:   # screen video of the Gazebo view (same capture as seq.py)
            ff = subprocess.Popen(['ffmpeg', '-y', '-loglevel', 'error', '-f', 'x11grab', '-draw_mouse', '0', '-framerate', '15',
                                   '-video_size', '1918x988', '-i', os.environ.get('DISPLAY', ':0') + '+1,62',
                                   '-vf', 'scale=960:-2', '-c:v', 'libx264', '-preset', 'ultrafast', '-crf', '30', '-pix_fmt', 'yuv420p',
                                   os.path.join(a.out, f'trial{k:02d}.mp4')], stdin=subprocess.PIPE,
                                  stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception:
            ff = None
        stop_t = None
        if a.stop_on_goal:   # end the episode when one roll is done: stop the policy, keep watching until run_s
            t_end = st['now'] + a.run_s
            live = None
            while st['now'] < t_end:
                rclpy.spin_once(node, timeout_sec=0.02)
                if stop_t is None and st['samples']:
                    live = live or R.Roll(st['samples'][0][1])
                    ang = live(st['last'][0])
                    se = R.spine(*st['last'][0])
                    q = st['last'][0]
                    if ang is not None and ang >= 330 and abs(se[2]) < 0.3 and 2 * (q[0] * q[2] - q[1] * q[3]) > 0.8:
                        stop_t = st['now'] - st['samples'][0][0]
                        os.killpg(proc.pid, signal.SIGINT)
        else:
            wait_sim(a.run_s)
        st['rec'] = False
        if ff:
            try:
                ff.communicate(b'q', timeout=15)
            except Exception:
                ff.kill()
        try:
            os.killpg(proc.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            proc.wait(timeout=20)
        except Exception:
            os.killpg(proc.pid, signal.SIGKILL)
        if not st['samples']:
            print(f'trial {k}: no data (policy did not start, see {log.name})', flush=True)
            continue
        rr = R.Roll(st['samples'][0][1])
        rows = [rr(q) for _, q in st['samples']]
        qe = st['samples'][-1][1]
        se = R.spine(*qe)
        along = se[0] * rr.ref[0] + se[1] * rr.ref[1]
        belly = 2 * (qe[0] * qe[2] - qe[1] * qe[3])
        res = dict(trial=k, max=max(rows), end=rows[-1], up=se[2], belly=belly, along=along,
                   n_cmd=st['n_cmd'] - n0, stop_t=stop_t)
        res['success'] = res['max'] >= 300 and 330 <= res['end'] <= 390 and abs(res['up']) < 0.3 and belly > 0.8
        n_ok += res['success']
        with open(os.path.join(a.out, 'results.jsonl'), 'a') as f:
            f.write(json.dumps(dict(time=time.strftime('%F %T'), model=a.model_path, **res)) + '\n')
        print(f'trial {k}: max {res["max"]:.0f} end {res["end"]:.0f} up {se[2]:+.2f} belly {belly:+.2f} '
              f'cmds {res["n_cmd"]} {"SUCCESS" if res["success"] else "fail"}  ({n_ok}/{k + 1})', flush=True)
    print(f'success {n_ok}/{a.trials}', flush=True)


if __name__ == '__main__':
    main()
