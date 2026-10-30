#!/usr/bin/env python3
"""One-leg kick backward roll: the heavy right leg (+2 kg shoe) kicks up over the head while the left
foot stays planted and pushes the floor (one-leg bridge), so the floor reaction lifts the hips.
  python3 kick.py kick                 # run
  python3 kick.py kick k3.lhip=0.5     # override (see seq.py)
Extra groups: lhip rhip lknee rknee lank rank (one leg each)."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import roll_trial as R  # noqa: E402
import seq as S         # noqa: E402

for k, s in (('l', 'left'), ('r', 'right')):
    R.GROUPS[k + 'hip'] = (s + '_hip_pitch_joint', None, 1)
    R.GROUPS[k + 'knee'] = (s + '_knee_joint', None, 1)
    R.GROUPS[k + 'ank'] = (s + '_ankle_pitch_joint', None, 1)
    R.LIMITS[k + 'hip'] = (-2.53, 2.87)
    R.LIMITS[k + 'knee'] = (-0.08, 2.87)
    R.LIMITS[k + 'ank'] = (-0.87, 0.52)

PLANT = dict(lhip=-1.05, lknee=2.6, lank=0.0)          # left foot flat on the floor under the knee
S.SEQS['kick'] = [
    (1.0, dict(PLANT, rhip=0.2, rknee=0.0, rank=0.0, sp=-2.8, el=1.5), None),  # 1 wind-up: right leg long, arms overhead
    (0.3, dict(), None),                                                       # 2 hold
    (0.25, dict(rhip=-2.53, rknee=0.3, lhip=0.3, lknee=2.0, lank=0.3, sp=-3.08), None),  # 3 kick right + push left
    (0.3, dict(lhip=-2.53, lknee=1.5, rknee=1.0, waist=0.52, **S.EAR), ('>=', 35, 'early')),  # 4 left follows, hands to ears
    (0.15, dict(sp=-2.6, el=1.2), ('>=', 70, 'early')),                        # 5 push with the hands
    (0.5, dict(lknee=2.0, rknee=2.0), None),                                   # 6 tuck, land on the knees
]

if __name__ == '__main__':
    S.main()
