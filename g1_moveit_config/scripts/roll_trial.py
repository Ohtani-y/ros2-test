#!/usr/bin/env python3
"""Backward-roll trials for G1 in Gazebo (trial and error).

Each trial resets G1 to lying on its back, sends one fast keyframe trajectory straight to
g1_controller (no MoveIt, so the swing keeps its momentum), records the pelvis orientation
and prints how far the body rotated backward over the shoulders:
  roll = 0: lying on the back, 90: on the shoulders (hips up), 180: rolled over.

Motion: wind up -> throw (legs swing up and over the head, arms slam) -> hands beside the
ears (STEP1 arm pose) and push off the floor -> tuck and land on the knees.
With push_roll > 0 the push starts when the measured roll reaches push_roll (closed loop).

  python3 roll_trial.py                               # default motion (P below)
  python3 roll_trial.py throw=0.25 knee=0.3           # override parameters
  python3 roll_trial.py throw=0.2,0.3 knee=0,1.5      # comma lists -> every combination
  python3 roll_trial.py --no-reset ...                # start from the current pose
Every trial is appended to /tmp/roll_trials/log.jsonl; the best one is kept in /tmp/roll_trials/best.json.
"""
import itertools
import json
import math
import os
import subprocess
import sys
import time

JOINTS = [
    'left_hip_pitch_joint', 'left_hip_roll_joint', 'left_hip_yaw_joint', 'left_knee_joint',
    'left_ankle_pitch_joint', 'left_ankle_roll_joint',
    'right_hip_pitch_joint', 'right_hip_roll_joint', 'right_hip_yaw_joint', 'right_knee_joint',
    'right_ankle_pitch_joint', 'right_ankle_roll_joint',
    'waist_yaw_joint', 'waist_roll_joint', 'waist_pitch_joint',
    'left_shoulder_pitch_joint', 'left_shoulder_roll_joint', 'left_shoulder_yaw_joint', 'left_elbow_joint',
    'left_wrist_roll_joint', 'left_wrist_pitch_joint', 'left_wrist_yaw_joint',
    'right_shoulder_pitch_joint', 'right_shoulder_roll_joint', 'right_shoulder_yaw_joint', 'right_elbow_joint',
    'right_wrist_roll_joint', 'right_wrist_pitch_joint', 'right_wrist_yaw_joint',
]
# joint groups: (left joint, right joint, right sign)
GROUPS = {
    'hip': ('left_hip_pitch_joint', 'right_hip_pitch_joint', 1),
    'knee': ('left_knee_joint', 'right_knee_joint', 1),
    'waist': ('waist_pitch_joint', None, 1),
    'sp': ('left_shoulder_pitch_joint', 'right_shoulder_pitch_joint', 1),
    'sr': ('left_shoulder_roll_joint', 'right_shoulder_roll_joint', -1),
    'sy': ('left_shoulder_yaw_joint', 'right_shoulder_yaw_joint', -1),
    'el': ('left_elbow_joint', 'right_elbow_joint', 1),
    'wr': ('left_wrist_roll_joint', 'right_wrist_roll_joint', -1),
    'wy': ('left_wrist_yaw_joint', 'right_wrist_yaw_joint', -1),
    'hr': ('left_hip_roll_joint', 'right_hip_roll_joint', 1),     # same sign: both legs to the robot's left (+)
    'wyaw': ('waist_yaw_joint', None, 1),
    'wroll': ('waist_roll_joint', None, 1),
}
LIMITS = {'hip': (-2.53, 2.87), 'knee': (-0.08, 2.87), 'waist': (-0.52, 0.52), 'sp': (-3.08, 2.67),
          'sr': (-1.58, 2.25), 'sy': (-2.61, 2.61), 'el': (-1.04, 2.09), 'wr': (-1.97, 1.97), 'wy': (-1.61, 1.61),
          'hr': (-0.52, 0.52), 'wyaw': (-2.6, 2.6), 'wroll': (-0.52, 0.52)}
# G1 signs: hip - = leg up / over the head, knee + = bend, waist + = curl the trunk,
#           shoulder pitch - = arm up / overhead, + = arm back (into the floor when lying),
#           shoulder roll + = arm out (left), elbow ~1.5 = straight, ~0 = bent 90 deg, - = bent more
P = dict(
    # timing (s)
    prep=1.0,         # zero pose -> wind-up pose
    hold=0.3,         # hold the wind-up pose
    load=0.0,         # optional: press the legs into the floor first (0 = skip)
    throw=0.25,       # leg swing duration
    arm_lead=0.05,    # arms start the slam this much earlier than the legs (negative = later)
    arm_throw=0.20,   # arm slam duration
    arm_with_load=0,  # 1: start the arm slam at the beginning of the load phase (with the leg slam)
    push_roll=0.0,    # >0: hands go to the ears when the roll reaches this angle (deg), instead of at push_t
    push_t=0.45,      # after the throw starts: move the hands beside the ears (open loop)
    push_move=0.20,
    push=0.15,        # push off the floor (elbow extension)
    land_t=0.9,       # after the throw starts (or after the push trigger): tuck the legs
    after=2.5,        # keep recording after the motion
    # wind-up (furikaburi) pose
    wind_sp=-2.8, wind_el=1.5, wind_hip=0.0, wind_knee=0.0, wind_waist=0.0,
    load_hip=0.3, load_knee=0.0,
    # throw targets
    hip=-2.53, knee=0.0, throw_waist=0.0,
    press_sp=0.5, press_el=1.5, sr=0.15,
    side_hr=0.0, side_wyaw=0.0, side_wroll=0.0,   # throw to one side (over one shoulder): + = robot's left
    # hands beside the ears (STEP1 arm pose), then push
    ear_sp=-2.05, ear_sr=0.74, ear_sy=0.30, ear_el=-0.72, ear_wr=1.38, ear_wy=1.23,
    push_sp=-2.6, push_el=1.2, push_waist=0.52,
    # landing
    land_hip=-2.53, land_knee=2.0,
)


def interp(points, t):
    if t <= points[0][0]:
        return points[0][1]
    for (t0, v0), (t1, v1) in zip(points, points[1:]):
        if t <= t1:
            return v1 if t1 <= t0 else v0 + (v1 - v0) * (t - t0) / (t1 - t0)
    return points[-1][1]


def mono(points):
    """Keep key times non-decreasing (later keys win if parameters overlap)."""
    out, last = [], 0.0
    for t, v in points:
        last = max(last, t)
        out.append((last, v))
    return out


def clip(tl):
    return {g: [(t, min(max(v, LIMITS[g][0]), LIMITS[g][1])) for t, v in mono(k)] for g, k in tl.items()}


def swing_start(p):
    return p['prep'] + p['hold'] + p['load'] + max(p['arm_lead'], 0.0)


def timelines(p, with_push=True):
    """Per-group (time, value) keys. Time 0 = trajectory start (from the zero pose)."""
    w = p['prep']
    a = w + p['hold']
    b = a + p['load']
    s = b + max(p['arm_lead'], 0.0)         # leg swing starts
    sa = b + max(-p['arm_lead'], 0.0)       # arm slam starts
    if p['arm_with_load']:
        sa -= p['load']                     # arms slam together with the legs pressing down
    e = s + p['throw']
    hip = [(0, 0.0), (w, p['wind_hip']), (a, p['wind_hip'])]
    knee = [(0, 0.0), (w, p['wind_knee']), (a, p['wind_knee'])]
    if p['load'] > 0:
        hip += [(b, p['load_hip']), (s, p['load_hip'])]
        knee += [(b, p['load_knee']), (s, p['load_knee'])]
    else:
        knee += [(s, p['wind_knee'])]
    tl = {
        'hip': hip + [(e, p['hip'])],
        'knee': knee + [(e, p['knee'])],
        'waist': [(0, 0.0), (w, p['wind_waist']), (s, p['wind_waist']), (e, p['throw_waist'])],
        'sp': [(0, 0.0), (w, p['wind_sp']), (sa, p['wind_sp']), (sa + p['arm_throw'], p['press_sp'])],
        'sr': [(0, 0.0), (w, p['sr'])],
        'sy': [(0, 0.0)], 'wr': [(0, 0.0)], 'wy': [(0, 0.0)],
        'hr': [(0, 0.0), (s, 0.0), (e, p['side_hr'])],
        'wyaw': [(0, 0.0), (s, 0.0), (e, p['side_wyaw'])],
        'wroll': [(0, 0.0), (s, 0.0), (e, p['side_wroll'])],
        'el': [(0, 0.0), (w, p['wind_el']), (sa, p['wind_el']), (sa + p['arm_throw'], p['press_el'])],
    }
    if with_push:
        tl = add_push(tl, p, s + p['push_t'], s + p['land_t'])
    return clip(tl)


def add_push(tl, p, pt, lt):
    """Append: hands to the ears at pt, push, tuck the legs at lt (absolute times)."""
    tl = {g: list(k) for g, k in tl.items()}
    pe = pt + p['push_move']
    pp = pe + p['push']
    for g, v in (('sp', p['ear_sp']), ('sr', p['ear_sr']), ('sy', p['ear_sy']), ('el', p['ear_el']),
                 ('wr', p['ear_wr']), ('wy', p['ear_wy']), ('waist', p['push_waist'])):
        tl[g] += [(pt, tl[g][-1][1]), (pe, v)]
    tl['sp'] += [(pp, p['push_sp'])]
    tl['el'] += [(pp, p['push_el'])]
    for g in ('hip', 'knee'):
        tl[g] += [(lt, tl[g][-1][1])]
    tl['hip'] += [(lt + 0.3, p['land_hip'])]
    tl['knee'] += [(lt + 0.3, p['land_knee'])]
    return tl


def sample(tl, t0, t1, dt=0.02):
    """Trajectory points (time from t0, positions[29]) for absolute times t0 < t <= t1."""
    out = []
    n = max(1, int(math.ceil((t1 - t0) / dt)))
    for i in range(1, n + 1):
        t = t0 + i * dt
        q = dict.fromkeys(JOINTS, 0.0)
        for g, keys in tl.items():
            lj, rj, sign = GROUPS[g]
            v = interp(keys, t)
            q[lj] = v
            if rj:
                q[rj] = sign * v
        out.append((t - t0, [q[j] for j in JOINTS]))
    return out


def end_time(tl):
    return max(k[-1][0] for k in tl.values())


def spine(qx, qy, qz, qw):
    """Pelvis z axis (pelvis -> head) in world coordinates."""
    return (2 * (qx * qz + qy * qw), 2 * (qy * qz - qx * qw), 1 - 2 * (qx * qx + qy * qy))


class Roll:
    """Incremental backward-roll angle (deg) of the pelvis, unwrapped."""
    def __init__(self, q0):
        s0 = spine(*q0)
        h = math.hypot(s0[0], s0[1]) or 1.0
        self.ref = (s0[0] / h, s0[1] / h)
        self.prev = None
        self.off = 0.0

    def __call__(self, q):
        s = spine(*q)
        ang = math.degrees(math.atan2(-s[2], s[0] * self.ref[0] + s[1] * self.ref[1]))
        if self.prev is not None:
            while ang + self.off - self.prev > 180:
                self.off -= 360
            while ang + self.off - self.prev < -180:
                self.off += 360
        self.prev = ang + self.off
        return self.prev


def summarize(rows, t_swing):
    rmax = max(rows, key=lambda r: r[1])
    end = rows[-1][1]
    verdict = ('ROLLED OVER' if end > 150 else 'on shoulders' if rmax[1] > 60 else
               'tipped' if rmax[1] > 25 else 'stayed on back')
    score = rmax[1] + 0.5 * max(end, 0.0)
    return dict(roll_max=round(rmax[1], 1), t_max=round(rmax[0] - t_swing, 2), roll_end=round(end, 1),
                z_max=round(max(r[2] for r in rows), 3), score=round(score, 1), verdict=verdict)


def parse(args):
    fixed, lists, ranges = {}, {}, {}
    for a in args:
        k, v = a.split('=', 1)
        if k not in P:
            raise SystemExit(f'unknown parameter {k}; known: {", ".join(P)}')
        if ':' in v:
            ranges[k] = tuple(float(x) for x in v.split(':'))
        elif ',' in v:
            lists[k] = [float(x) for x in v.split(',')]
        else:
            fixed[k] = float(v)
    return fixed, lists, ranges


def main():
    import rclpy
    from builtin_interfaces.msg import Duration
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

    argv = sys.argv[1:]
    reset = '--no-reset' not in argv
    n_random = 0
    if '--random' in argv:
        i = argv.index('--random')
        n_random = int(argv[i + 1])
        argv = argv[:i] + argv[i + 2:]
    fixed, lists, ranges = parse([a for a in argv if not a.startswith('--')])

    rclpy.init()
    node = Node('roll_trial')
    pub = node.create_publisher(JointTrajectory, '/g1_controller/joint_trajectory', 10)
    st = {'samples': [], 'now': None, 'rec': False, 'last': None}

    def on_odom(m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        o, p = m.pose.pose.orientation, m.pose.pose.position
        st['now'] = t
        st['last'] = ((o.x, o.y, o.z, o.w), (p.x, p.y, p.z))
        if st['rec']:
            st['samples'].append((t, st['last'][0], st['last'][1]))

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
        msg.joint_names = JOINTS
        for t, q in points:
            pt = JointTrajectoryPoint()
            pt.positions = [float(x) for x in q]
            pt.time_from_start = Duration(sec=int(t), nanosec=int(round((t % 1) * 1e9)) % 1000000000)
            msg.points.append(pt)
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.01)

    def do_reset():
        for _ in range(4):
            send([(1.0, [0.0] * len(JOINTS))])
            wait_sim(1.5)
            subprocess.run(['ign', 'service', '-s', '/world/ground/set_pose', '--reqtype', 'ignition.msgs.Pose',
                            '--reptype', 'ignition.msgs.Boolean', '--timeout', '3000', '--req',
                            'name: "g1", position: {x: 0, y: 0, z: 0.12}, '
                            'orientation: {x: 0, y: -0.70710678, z: 0, w: 0.70710678}'],
                           capture_output=True, timeout=10)
            wait_sim(2.5)
            sp = spine(*st['last'][0])
            if abs(sp[2]) < 0.1 and st['last'][1][2] < 0.2:
                return
            print(f'  reset retry (spine z={sp[2]:.2f}, pelvis z={st["last"][1][2]:.2f})', flush=True)

    os.makedirs('/tmp/roll_trials', exist_ok=True)
    best_path = '/tmp/roll_trials/best.json'
    best = json.load(open(best_path)) if os.path.exists(best_path) else None

    def run(p, label):
        nonlocal best
        if reset:
            do_reset()
        closed = p['push_roll'] > 0
        tl = timelines(p, with_push=not closed)
        t_swing = swing_start(p)
        st['samples'] = []
        st['rec'] = True
        t0 = st['now']
        roll = Roll(st['last'][0])
        trig = None
        if not closed:
            send(sample(tl, 0.0, end_time(tl)))
            wait_sim(end_time(tl) + p['after'])
        else:
            send(sample(tl, 0.0, end_time(tl) + 1.5))   # wind-up + throw, then hold the throw pose
            state = {'r': 0.0}

            def reached():
                state['r'] = roll(st['last'][0])
                return st['now'] - t0 >= t_swing and state['r'] >= p['push_roll']
            if wait_sim(end_time(tl) + 1.5, until=reached):
                trig = st['now'] - t0
                tl2 = clip(add_push({g: list(k) for g, k in tl.items()}, p, trig, trig + p['land_t']))
                send(sample(tl2, trig, end_time(tl2)))
                wait_sim(end_time(tl2) - trig + p['after'])
            else:
                wait_sim(p['after'])
        st['rec'] = False
        rr = Roll(st['samples'][0][1])
        rows = [(t - t0, rr(q), pos[2]) for t, q, pos in st['samples']]
        res = summarize(rows, t_swing)
        res['push_at'] = None if trig is None else round(trig - t_swing, 2)
        print(f'{label:<40} {res["roll_max"]:>6.1f} {res["t_max"]:>5.2f} {res["roll_end"]:>6.1f} '
              f'{res["z_max"]:>5.3f} {str(res["push_at"]):>5} {res["verdict"]}', flush=True)
        rec = dict(time=time.strftime('%H:%M:%S'), params=p, label=label, **res)
        with open('/tmp/roll_trials/log.jsonl', 'a') as f:
            f.write(json.dumps(rec) + '\n')
        with open('/tmp/roll_trials/last.csv', 'w') as f:
            f.write('t,roll_deg,pelvis_z\n' + ''.join(f'{t:.3f},{r:.1f},{z:.3f}\n' for t, r, z in rows))
        if best is None or res['score'] > best['score']:
            best = rec
            json.dump(best, open(best_path, 'w'), indent=1)

    print(f'{"trial":<40} {"max":>6} {"at":>5} {"end":>6} {"z":>5} {"push":>5} verdict')
    base = dict(P, **fixed)
    combos = [dict(zip(lists, c)) for c in itertools.product(*lists.values())] or [{}]
    if n_random:
        import random
        combos = [{k: round(random.uniform(*r), 3) for k, r in ranges.items()} for _ in range(n_random)]
    for i, c in enumerate(combos, 1):
        run(dict(base, **c), f'{i} ' + (' '.join(f'{k}={v:g}' for k, v in c.items()) or '(base)'))
    if best:
        print('best so far:', best['label'], best['roll_max'], best['roll_end'], best['verdict'])


if __name__ == '__main__':
    main()
