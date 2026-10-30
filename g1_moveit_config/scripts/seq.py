#!/usr/bin/env python3
"""Keyframe sequences for G1 in Gazebo with roll-angle triggers (closed loop).

A sequence is a list of keyframes: (seconds to reach, {group: value}, trigger).
trigger = None, or ('>=', deg) / ('<=', deg): after the previous keyframes have finished, wait
until the pelvis roll satisfies it (max 3 s); ('>=', deg, 'early') may fire before they finish.
Unlisted groups keep their value.
Roll: 0 = lying on the back, -90 = sitting up, +90 = on the shoulders, 180 = rolled over.

  python3 seq.py rockroll                 # run a sequence
  python3 seq.py rockroll k3.hip=-2.4     # override keyframe 3 value (1-based)
  python3 seq.py rockroll k2.t=0.4        # override keyframe 2 duration
"""
import json
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import roll_trial as R   # noqa: E402

D = math.radians
PLANNER = None   # optional: f(node, q_start, q_goal, duration) -> [(t, q29)], e.g. MoveIt (moveit_roll.py)
EAR = dict(sp=-2.05, sr=0.74, sy=0.30, el=-0.72, wr=1.38, wy=1.23)   # STEP1 arms

SEQS = {
    # sit up first, then throw the trunk back (rock) with momentum, legs over, hands beside the ears, push, land
    'rockroll': [
        (1.5, dict(hip=-1.5, knee=0.0, waist=0.4, sp=-1.3, el=1.3), None),             # 1 sit up
        (0.6, dict(hip=-2.1, knee=2.3, waist=0.5), None),                              # 2 tuck while sitting
        (0.25, dict(hip=-1.4, waist=-0.2, **EAR), ('<=', -50)),                        # 3 throw the trunk back
        (0.2, dict(hip=-2.53, knee=1.0, waist=0.52), ('>=', -10, 'early')),            # 4 back hits floor: legs over
        (0.15, dict(sp=-2.6, el=1.2), ('>=', 50, 'early')),                            # 5 push with the hands
        (0.5, dict(knee=2.0), None),                                                   # 6 land on the knees
    ],
}


def parse_overrides(args, seq):
    seq = [(t, dict(pose), trig) for t, pose, trig in seq]
    for a in args:
        k, v = a.split('=', 1)
        idx, key = k.split('.', 1)
        i = int(idx[1:]) - 1
        t, pose, trig = seq[i]
        if key == 't':
            t = float(v)
        elif key == 'trig':
            trig = None if v == 'none' else ((v[:2], float(v[2:].rstrip('e')), 'early') if v.endswith('e')
                                             else (v[:2], float(v[2:])))
        else:
            pose[key] = float(v)
        seq[i] = (t, pose, trig)
    return seq


def main():
    import rclpy
    from builtin_interfaces.msg import Duration
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

    name = sys.argv[1]
    reset = '--no-reset' not in sys.argv     # --no-reset: start from the current state (chain rolls)
    seq = parse_overrides([a for a in sys.argv[2:] if not a.startswith('--')], SEQS[name])
    wait_max, after = 3.0, 2.5

    rclpy.init()
    node = Node('roll_seq')
    pub = node.create_publisher(JointTrajectory, '/g1_controller/joint_trajectory', 10)
    act_pub = node.create_publisher(JointTrajectory, '/roll/action', 10)   # commanded target, 50 Hz (for learning)
    st = {'now': None, 'last': None, 'rec': False, 'samples': []}

    def on_odom(m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        o, p = m.pose.pose.orientation, m.pose.pose.position
        st['now'] = t
        st['last'] = ((o.x, o.y, o.z, o.w), (p.x, p.y, p.z))
        if st['rec']:
            st['samples'].append((t, st['last'][0], st['last'][1]))
        if st.get('act_on') and t - st.get('act_t', -1.0) >= 0.0199:
            st['act_t'] = t
            q = [0.0] * len(R.JOINTS)
            if st.get('sent'):
                ts, pts = st['sent'][-1]
                q = pts[0][1]
                for tt, qq in pts:
                    if ts + tt > t:
                        break
                    q = qq
            am = JointTrajectory()
            am.joint_names = R.JOINTS
            ap = JointTrajectoryPoint()
            ap.positions = [float(x) for x in q]
            am.points.append(ap)
            act_pub.publish(am)

    node.create_subscription(Odometry, '/model/g1/odometry', on_odom, 50)
    while pub.get_subscription_count() == 0 or st['now'] is None:
        rclpy.spin_once(node, timeout_sec=0.1)

    def wait_sim(sec, until=None):
        t_end = st['now'] + sec
        while st['now'] < t_end:
            rclpy.spin_once(node, timeout_sec=0.02)
            if until is not None and until():
                return True
        return False

    def send(points):
        msg = JointTrajectory()
        msg.joint_names = R.JOINTS
        for t, q in points:
            pt = JointTrajectoryPoint()
            pt.positions = [float(x) for x in q]
            pt.time_from_start = Duration(sec=int(t), nanosec=int(round((t % 1) * 1e9)) % 1000000000)
            msg.points.append(pt)
        pub.publish(msg)
        st.setdefault('sent', []).append((st['now'], [(t, list(q)) for t, q in points]))
        rclpy.spin_once(node, timeout_sec=0.01)

    def to_q(pose):
        q = dict.fromkeys(R.JOINTS, 0.0)
        for g, v in pose.items():
            lo, hi = R.LIMITS[g]
            v = min(max(v, lo), hi)
            lj, rj, sign = R.GROUPS[g]
            q[lj] = v
            if rj:
                q[rj] = sign * v
        return [q[j] for j in R.JOINTS]

    plans = {}
    if PLANNER is not None:      # plan every keyframe segment in advance
        c = {}
        for k, (dur, pose, _) in enumerate(seq):
            tg = dict(c)
            tg.update(pose)
            plans[k] = PLANNER(node, to_q(c), to_q(tg), dur)
            c = tg

    # reset to lying on the back
    for _ in range(4 if reset else 0):
        send([(1.0, [0.0] * len(R.JOINTS))])
        wait_sim(1.5)
        import subprocess
        subprocess.run(['ign', 'service', '-s', '/world/ground/set_pose', '--reqtype', 'ignition.msgs.Pose',
                        '--reptype', 'ignition.msgs.Boolean', '--timeout', '3000', '--req',
                        'name: "g1", position: {x: 0, y: 0, z: 0.12}, '
                        'orientation: {x: 0, y: -0.70710678, z: 0, w: 0.70710678}'],
                       capture_output=True, timeout=10)
        wait_sim(2.5)
        sp = R.spine(*st['last'][0])
        if abs(sp[2]) < 0.1 and st['last'][1][2] < 0.2:
            break

    bagp = None
    if os.environ.get('ROLL_BAG'):     # ros2 bag of this run: joint states, commanded action, pelvis odometry
        import subprocess as sp2
        bagp = sp2.Popen(['ros2', 'bag', 'record', '--use-sim-time', '-o', os.environ['ROLL_BAG'],
                          '/joint_states', '/roll/action', '/model/g1/odometry'],
                         stdout=sp2.DEVNULL, stderr=sp2.DEVNULL, start_new_session=True)
        st['sent'] = []
        st['act_on'] = True
        t_w = time.time() + 2.0
        while time.time() < t_w:
            rclpy.spin_once(node, timeout_sec=0.05)
    st['act_on'] = True
    st['samples'] = []
    st['sent'] = []
    st['rec'] = True
    import subprocess
    os.makedirs('/tmp/roll_trials', exist_ok=True)
    try:   # screen video of the Gazebo view (kept only when the robot rolls over)
        ff = subprocess.Popen(['ffmpeg', '-y', '-loglevel', 'error', '-f', 'x11grab', '-framerate', '15',
                               '-video_size', '860x640', '-i', os.environ.get('DISPLAY', ':0') + '+0,60',
                               '-c:v', 'libx264', '-preset', 'ultrafast', '-crf', '30', '-pix_fmt', 'yuv420p',
                               '/tmp/roll_trials/seq_video.mp4'], stdin=subprocess.PIPE, stdout=subprocess.DEVNULL,
                              stderr=subprocess.DEVNULL)
    except Exception:
        ff = None
    t0 = st['now']
    roll = R.Roll(st['last'][0])
    cur = {}          # current commanded group values
    seg_end = 0.0     # time (from t0) when the last sent segment finishes
    log = []
    i = 0
    while i < len(seq):
        # segment: keyframes i..j-1 until the next trigger
        j = i + 1
        while j < len(seq) and seq[j][2] is None:
            j += 1
        if seq[i][2] is not None:
            op, ang = seq[i][2][:2]
            if len(seq[i][2]) < 3:
                wait_sim(max(0.0, seg_end - (st['now'] - t0)))
            ok = wait_sim(wait_max, until=lambda: (roll(st['last'][0]) >= ang) if op == '>=' else
                          (roll(st['last'][0]) <= ang))
            log.append((round(st['now'] - t0, 2), f'k{i + 1} trigger {op}{ang}', ok, round(roll.prev or 0, 1)))
            if not ok:
                print(f'  trigger k{i + 1} {op}{ang} not reached (roll {roll.prev:.1f}); stop', flush=True)
                break
        pts, t = [], 0.0
        start = dict(cur)
        for k in range(i, j):
            dur, pose, _ = seq[k]
            target = dict(start)
            target.update(pose)
            if plans.get(k):
                pts += [(t + dt, q) for dt, q in plans[k]]
            else:
                n = max(1, int(round(dur / 0.02)))
                for s in range(1, n + 1):
                    a = s / n
                    mix = {g: start.get(g, 0.0) + a * (target[g] - start.get(g, 0.0)) for g in target}
                    pts.append((t + s * dur / n, to_q(mix)))
            t += dur
            start = target
        send(pts)
        seg_end = st['now'] - t0 + t
        cur = start
        log.append((round(st['now'] - t0, 2), f'sent k{i + 1}-k{j}', True, round(roll(st['last'][0]), 1)))
        if j >= len(seq):
            wait_sim(t + after, until=lambda: roll(st['last'][0]) is None)
        else:
            wait_sim(0.0)
        i = j
    st['rec'] = False
    if bagp:
        import signal
        wait_sim(0.5)
        os.killpg(bagp.pid, signal.SIGINT)
        try:
            bagp.wait(timeout=30)
        except Exception:
            os.killpg(bagp.pid, signal.SIGKILL)
    if ff:
        try:
            ff.communicate(b'q', timeout=15)
        except Exception:
            ff.kill()
    rr = R.Roll(st['samples'][0][1])
    rows = [(tt - t0, rr(q), pos[2]) for tt, q, pos in st['samples']]
    rmax = max(rows, key=lambda r: r[1])
    rmin = min(rows, key=lambda r: r[1])
    print(f'{name} {" ".join(sys.argv[2:])}')
    for e in log:
        print('  ', e)
    print(f'  roll min {rmin[1]:.1f} at {rmin[0]:.2f}s, max {rmax[1]:.1f} at {rmax[0]:.2f}s, end {rows[-1][1]:.1f}', flush=True)
    sz = [R.spine(*q)[2] for _, q, _ in st['samples']]
    p0, p1 = st['samples'][0][2], st['samples'][-1][2]
    dx = (p1[0] - p0[0]) * rr.ref[0] + (p1[1] - p0[1]) * rr.ref[1]
    se = R.spine(*st['samples'][-1][1])
    qe = st['samples'][-1][1]
    print(f'  inverted {-min(sz):.2f}  hips moved {dx:+.2f} m to the head side  end spine: along {se[0] * rr.ref[0] + se[1] * rr.ref[1]:+.2f} up {se[2]:+.2f}  belly up {2 * (qe[0] * qe[2] - qe[1] * qe[3]):+.2f}', flush=True)
    os.makedirs('/tmp/roll_trials', exist_ok=True)
    with open('/tmp/roll_trials/seq_last.csv', 'w') as f:
        f.write('t,roll_deg,pelvis_z\n' + ''.join(f'{a:.3f},{b:.1f},{c:.3f}\n' for a, b, c in rows))
    with open('/tmp/roll_trials/seq_log.jsonl', 'a') as f:
        f.write(json.dumps(dict(time=time.strftime('%H:%M:%S'), name=name, args=sys.argv[2:], min=rmin[1],
                                max=rmax[1], end=rows[-1][1], inv=-min(sz), dx=dx, along=se[0] * rr.ref[0] + se[1] * rr.ref[1], up=se[2], belly=2 * (qe[0] * qe[2] - qe[1] * qe[3]))) + '\n')

    if rmax[1] >= 180:      # went over the head: always record (parameters, code snapshot, data, model)
        import glob
        import shutil
        top = '/workspace/g1_moveit_config/roll_records'
        rec = os.path.join(top, time.strftime('%Y%m%d_%H%M%S') + f'_{name}_{rmax[1]:.0f}deg')
        os.makedirs(rec, exist_ok=True)
        for f in glob.glob(os.path.join(os.path.dirname(os.path.abspath(__file__)), '*.py')):
            shutil.copy(f, rec)
        urdf = os.path.realpath('/workspace/g1_gazebo/urdf/g1_29dof_gazebo.urdf')
        if os.path.exists(urdf):
            shutil.copy(urdf, rec)
        if os.path.exists('/tmp/roll_trials/seq_video.mp4'):
            shutil.move('/tmp/roll_trials/seq_video.mp4', os.path.join(rec, 'video.mp4'))
        cmd = f'python3 {os.path.basename(sys.argv[0])} {name} ' + ' '.join(f"'{a}'" for a in sys.argv[2:])
        along = se[0] * rr.ref[0] + se[1] * rr.ref[1]
        belly = 2 * (qe[0] * qe[2] - qe[1] * qe[3])
        json.dump(dict(time=time.strftime('%Y-%m-%d %H:%M:%S'), name=name, args=sys.argv[2:], rerun=f'cd {rec} && {cmd}',
                       metrics=dict(min=rmin[1], max=rmax[1], t_max=rmax[0], end=rows[-1][1], inverted=-min(sz), dx=dx,
                                    along=along, up=se[2], belly=belly),
                       seq=seq, log=log), open(os.path.join(rec, 'record.json'), 'w'), indent=1)
        with open(os.path.join(rec, 'roll.csv'), 'w') as f:
            f.write('t,roll_deg,pelvis_z\n' + ''.join(f'{a:.3f},{b:.1f},{c:.3f}\n' for a, b, c in rows))
        with open(os.path.join(rec, 'traj.csv'), 'w') as f:
            f.write('t,' + ','.join(R.JOINTS) + '\n')
            for ts, pts in st['sent']:
                for t, q in pts:
                    f.write(f'{ts - t0 + t:.3f},' + ','.join(f'{v:.4f}' for v in q) + '\n')
        with open(os.path.join(rec, 'rerun.sh'), 'w') as f:
            f.write('#!/bin/bash\ncd "$(dirname "$0")"\n' + cmd + '\n')
        os.chmod(os.path.join(rec, 'rerun.sh'), 0o755)
        with open(os.path.join(top, 'index.txt'), 'a') as f:
            f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')}  max {rmax[1]:5.0f}  end {rows[-1][1]:5.0f}  "
                    f"belly {belly:+.2f}  {os.path.basename(rec)}  {cmd}\n")
        print(f'  RECORDED -> {rec}', flush=True)


if __name__ == '__main__':
    main()
