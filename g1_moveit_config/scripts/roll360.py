#!/usr/bin/env python3
"""Backward roll that ends in the start posture (lying on the back, all joints 0), so it can be repeated.
  python3 roll360.py roll360 [kN.key=value ...] [--no-reset]"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import pump3 as P3  # noqa: E402

S = P3.S
EAR = S.EAR
legs = P3.legs
ZERO = dict(sp=0.0, sr=0.0, sy=0.0, el=0.0, wr=0.0, wy=0.0, waist=0.0, lhip=0.0, rhip=0.0, lknee=0.0, rknee=0.0)
S.SEQS['roll360'] = [
    (0.6, dict(EAR), None),                                    # 1 hands beside the ears
    (1.0, legs(-1.57, 0.0), None),                             # 2 legs up (the upper body sits up)
    (0.3, dict(), None),                                       # 3 hold
    (0.2, legs(0.4, 0.0), None),                               # 4 slam the legs (trunk thrown back)
    (0.1, dict(legs(-2.3, 0.0), waist=0.52), None),            # 5 FAST kick, head forward
    (0.25, legs(-2.53, 1.0), ('>=', 30, 'early')),             # 6 fold over the head
    (0.15, dict(sp=-2.6, el=1.2), ('>=', 70, 'early')),        # 7 push with the hands
    (0.5, dict(lknee=2.0, rknee=2.0), None),                   # 8 tuck: comes up sitting (~270 deg)
    (0.6, dict(ZERO), ('>=', 260, 'early')),                   # 9 lie back down into the start posture
    (1.0, dict(), None),                                       # 10 hold
]

if __name__ == '__main__':
    S.main()
