#!/usr/bin/env python3
"""Backward roll (pump3, kick -2.3 rad) with landing variants.
  squat: tight tuck (knees max) and arms forward at the end, to stay on the feet
  python3 land.py squat [kN.key=value ...]"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import pump3 as P3  # noqa: E402

S = P3.S
EAR = S.EAR
legs = P3.legs
ROLL = [
    (0.6, dict(EAR), None),                                                    # 1 hands beside the ears
    (1.0, legs(-1.57, 0.0), None),                                             # 2 legs up (the upper body sits up)
    (0.3, dict(), None),                                                       # 3 hold
    (0.2, legs(0.4, 0.0), None),                                               # 4 slam the legs (trunk thrown back)
    (0.1, dict(legs(-2.3, 0.0), waist=0.52), None),                            # 5 FAST kick, head forward
    (0.25, legs(-2.53, 1.0), ('>=', 30, 'early')),                             # 6 fold over the head
    (0.15, dict(sp=-2.6, el=1.2), ('>=', 70, 'early')),                        # 7 push with the hands
]
S.SEQS['squat'] = ROLL + [
    (0.3, legs(-2.53, 2.8), None),                                             # 8 tight tuck: feet to the buttocks
    (0.3, dict(sp=-1.2, el=1.5, waist=0.52), ('>=', 240, 'early')),            # 9 arms forward, trunk forward
]

if __name__ == '__main__':
    S.main()
