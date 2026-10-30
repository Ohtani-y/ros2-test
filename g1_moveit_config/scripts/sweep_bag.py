#!/usr/bin/env python3
"""Sweep the 9 main control values of the MoveIt backward roll (moveit_roll.py roll) by -0.1 / +0.1 rad
(2^9 = 512 runs, left and right always the same). Every run is recorded with ros2 bag
(/joint_states, /roll/action, /model/g1/odometry); runs that roll all the way back to the start pose
are copied to OUT/success/ (input for the LeRobot conversion).
  python3 sweep_bag.py [--start N] [--out DIR]
Results: OUT/results.jsonl (one line per run)."""
import argparse
import itertools
import json
import os
import shutil
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
SEQLOG = '/tmp/roll_trials/seq_log.jsonl'
# (name, base value, keyframes it applies to, groups); keyframes of 'roll':
# k1 LEGS_UP, k2 hold, k3 SLAM, k4 KICK, k5 FOLD, k6 PUSH, k7 TUCK, k8 ZERO, k9 hold
KNOBS = [
    ('LEGS_UP.hip', -1.57, (1, 2), ('lhip', 'rhip')),
    ('SLAM.hip', 0.40, (3,), ('lhip', 'rhip')),
    ('KICK.hip', -2.30, (4,), ('lhip', 'rhip')),
    ('KICK.waist', 0.52, (4, 5, 6, 7), ('waist',)),
    ('FOLD.hip', -2.53, (5, 6, 7), ('lhip', 'rhip')),
    ('FOLD.knee', 1.00, (5, 6), ('lknee', 'rknee')),
    ('PUSH.sp', -2.60, (6, 7), ('sp',)),
    ('PUSH.el', 1.20, (6, 7), ('el',)),
    ('TUCK.knee', 2.00, (7,), ('lknee', 'rknee')),
]
LIMIT = {'lhip': (-2.53, 2.87), 'rhip': (-2.53, 2.87), 'lknee': (-0.08, 2.87), 'rknee': (-0.08, 2.87),
         'waist': (-0.52, 0.52), 'sp': (-3.08, 2.67), 'el': (-1.04, 2.09)}


def overrides(signs):
    out, vals = [], {}
    for (name, base, kfs, groups), sg in zip(KNOBS, signs):
        v = base + 0.1 * sg
        for g in groups:
            v = min(max(v, LIMIT[g][0]), LIMIT[g][1])
            for k in kfs:
                out.append(f'k{k}.{g}={v:.3f}')
        vals[name] = round(v, 3)
    return out, vals


def success(r):
    return r['max'] >= 300 and 330 <= r['end'] <= 390 and abs(r['up']) < 0.3 and r['belly'] > 0.8


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--start', type=int, default=0)
    ap.add_argument('--out', default=os.path.expanduser('~/dataset/rosbags/g1_backroll'))
    a = ap.parse_args()
    for d in ('all', 'success', 'logs'):
        os.makedirs(os.path.join(a.out, d), exist_ok=True)
    combos = list(itertools.product((-1, 1), repeat=len(KNOBS)))
    env = dict(os.environ, DISPLAY=':99')          # no screen video during the sweep
    for i, signs in enumerate(combos):
        if i < a.start:
            continue
        ep = f'ep{i:03d}'
        bag = os.path.join(a.out, 'all', ep)
        if os.path.exists(bag):
            shutil.rmtree(bag)
        args, vals = overrides(signs)
        n0 = sum(1 for _ in open(SEQLOG)) if os.path.exists(SEQLOG) else 0
        t0 = time.time()
        with open(os.path.join(a.out, 'logs', ep + '.log'), 'w') as lf:
            subprocess.run([sys.executable, os.path.join(HERE, 'moveit_roll.py'), 'roll'] + args,
                           env=dict(env, ROLL_BAG=bag), stdout=lf, stderr=subprocess.STDOUT, timeout=600)
        lines = open(SEQLOG).read().splitlines() if os.path.exists(SEQLOG) else []
        r = json.loads(lines[-1]) if len(lines) > n0 else None
        ok = bool(r) and success(r) and os.path.exists(bag)
        if ok:
            dst = os.path.join(a.out, 'success', ep)
            if os.path.exists(dst):
                shutil.rmtree(dst)
            shutil.copytree(bag, dst)
        rec = dict(ep=ep, signs=signs, values=vals, success=ok, wall_s=round(time.time() - t0, 1),
                   **({k: r[k] for k in ('max', 'end', 'up', 'belly', 'inv')} if r else {}))
        with open(os.path.join(a.out, 'results.jsonl'), 'a') as f:
            f.write(json.dumps(rec) + '\n')
        n_ok = len(os.listdir(os.path.join(a.out, 'success')))
        print(f'{ep} {"OK " if ok else "NG "} max {r["max"] if r else None!s:>6} end {r["end"] if r else None!s:>6} '
              f'success {n_ok}/{i + 1}  {rec["wall_s"]} s', flush=True)


if __name__ == '__main__':
    main()
