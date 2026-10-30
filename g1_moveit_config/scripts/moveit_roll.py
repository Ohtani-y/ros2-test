#!/usr/bin/env python3
"""Backward roll from a few key poses; MoveIt (/plan_kinematic_path, group whole_body) plans the motion
between consecutive poses, the plans are joined into one trajectory per phase and run on g1_controller.
Ends in the start pose (lying on the back, all joints 0), so it can be repeated (--no-reset).
  python3 moveit_roll.py roll [kN.t=... kN.trig=...] [--no-reset]"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import seq as S          # noqa: E402
import roll_trial as R   # noqa: E402

for k, side in (('l', 'left'), ('r', 'right')):
    R.GROUPS[k + 'hip'] = (side + '_hip_pitch_joint', None, 1)
    R.GROUPS[k + 'knee'] = (side + '_knee_joint', None, 1)
    R.LIMITS[k + 'hip'] = (-2.53, 2.87)
    R.LIMITS[k + 'knee'] = (-0.08, 2.87)

GROUP = 'whole_body'
EAR = dict(sp=-2.05, sr=0.74, sy=0.30, el=-0.72, wr=1.38, wy=1.23)   # hands beside the ears (STEP1 arms)


def legs(hip, knee):
    return dict(lhip=hip, rhip=hip, lknee=knee, rknee=knee)


def P(*parts, **kw):
    out = dict(ZERO)
    for d in parts + (kw,):
        out.update(d)
    return out


# key poses (both sides the same)
ZERO = dict(sp=0.0, sr=0.0, sy=0.0, el=0.0, wr=0.0, wy=0.0, waist=0.0, **legs(0.0, 0.0))
POSES = {
    'ZERO': ZERO,                                                   # start / end: lying on the back
    'READY': P(EAR),                                     # hands beside the ears
    'LEGS_UP': P(EAR, legs(-1.57, 0.0)),               # legs up (the upper body sits up)
    'SLAM': P(EAR, legs(0.4, 0.0)),                    # legs slammed on the floor
    'KICK': P(EAR, legs(-2.3, 0.0), waist=0.52),       # fast kick over, head forward
    'FOLD': P(EAR, legs(-2.53, 1.0), waist=0.52),      # legs folded over the head
    'PUSH': P(EAR, legs(-2.53, 1.0), waist=0.52, sp=-2.6, el=1.2),   # push with the hands
    'TUCK': P(EAR, legs(-2.53, 2.0), waist=0.52, sp=-2.6, el=1.2),   # tuck, comes up sitting
}
# (seconds, pose, trigger on the pelvis roll angle)
S.SEQS['roll'] = [
    (1.0, POSES['LEGS_UP'], None),          # READY + LEGS_UP at the same time: hands to the ears while the legs go up
    (0.3, POSES['LEGS_UP'], None),
    (0.2, POSES['SLAM'], None),
    (0.1, POSES['KICK'], None),
    (0.25, POSES['FOLD'], ('>=', 30, 'early')),
    (0.15, POSES['PUSH'], ('>=', 70, 'early')),
    (0.5, POSES['TUCK'], None),
    (0.6, POSES['ZERO'], ('>=', 260, 'early')),
    (1.0, POSES['ZERO'], None),
]
_cli = {}


def moveit_plan(node, q0, q1, dur, dt=0.02):
    """MoveIt plan q0 -> q1, retimed to `dur` seconds and resampled every dt. None = no motion / failed."""
    import rclpy
    from moveit_msgs.msg import Constraints, JointConstraint
    from moveit_msgs.srv import GetMotionPlan
    if max(abs(a - b) for a, b in zip(q0, q1)) < 1e-6:
        return None                                      # hold: nothing to plan
    if 'c' not in _cli:
        _cli['c'] = node.create_client(GetMotionPlan, '/plan_kinematic_path')
        if not _cli['c'].wait_for_service(timeout_sec=10.0):
            raise SystemExit('MoveIt /plan_kinematic_path not available (start moveit.launch.py)')
    req = GetMotionPlan.Request()
    r = req.motion_plan_request
    r.group_name = GROUP
    r.start_state.is_diff = True
    r.start_state.joint_state.name = list(R.JOINTS)
    r.start_state.joint_state.position = [float(v) for v in q0]
    c = Constraints()
    for n, v in zip(R.JOINTS, q1):
        c.joint_constraints.append(JointConstraint(joint_name=n, position=float(v), tolerance_above=1e-3,
                                                   tolerance_below=1e-3, weight=1.0))
    r.goal_constraints = [c]
    r.num_planning_attempts = 1
    r.allowed_planning_time = 3.0
    r.max_velocity_scaling_factor = 1.0
    r.max_acceleration_scaling_factor = 1.0
    fut = _cli['c'].call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=15.0)
    res = fut.result()
    if res is None or res.motion_plan_response.error_code.val != 1:
        print('  MoveIt plan FAILED (linear fallback):', None if res is None else res.motion_plan_response.error_code.val)
        return None
    jt = res.motion_plan_response.trajectory.joint_trajectory
    idx = [jt.joint_names.index(n) if n in jt.joint_names else None for n in R.JOINTS]
    wps = [(p.time_from_start.sec + p.time_from_start.nanosec * 1e-9,
            [p.positions[i] if i is not None else q1[j] for j, i in enumerate(idx)]) for p in jt.points]
    T = wps[-1][0]
    if T <= 0:
        wps = [(i / max(1, len(wps) - 1), q) for i, (_, q) in enumerate(wps)]
        T = 1.0
    print(f'  MoveIt: {len(wps)} waypoints, {T:.2f} s -> retimed to {dur:.2f} s', flush=True)
    out, n, w = [], max(1, int(round(dur / dt))), 0
    for s in range(1, n + 1):
        tt = s / n * T
        while w < len(wps) - 2 and wps[w + 1][0] < tt:
            w += 1
        (ta, qa), (tb, qb) = wps[w], wps[min(w + 1, len(wps) - 1)]
        a = 1.0 if tb <= ta else min(max((tt - ta) / (tb - ta), 0.0), 1.0)
        out.append((s * dur / n, [x + a * (y - x) for x, y in zip(qa, qb)]))
    return out


S.PLANNER = moveit_plan

if __name__ == '__main__':
    S.main()
