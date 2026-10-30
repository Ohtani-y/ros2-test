#!/usr/bin/env python3
"""Symmetric kick-up rolls (both legs the same).
  pump2: arms overhead first, legs up, slam down, FAST kick over the head
  pump3: hands beside the ears (head height) from the start, legs up, slam down, FAST kick UP (rotate about
         the head), head forward (waist curl), then fold over, push, land
  python3 pump3.py pump3 [kN.key=value ...]"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import pump as P  # noqa: E402  (registers the one-leg groups and pump sequences)

S = P.S
EAR = S.EAR


def legs(hip, knee):
    return dict(lhip=hip, rhip=hip, lknee=knee, rknee=knee)


S.SEQS['pump2'] = [
    (0.6, dict(sp=-2.8, el=1.5), None),                                        # 1 arms overhead first
    (1.0, legs(-1.57, 0.0), None),                                              # 2 legs up
    (0.3, dict(), None),                                                        # 3 hold
    (0.2, legs(0.4, 0.0), None),                                                # 4 slam the legs on the floor
    (0.1, dict(legs(-2.53, 0.5), sp=-0.6), None),                               # 5 FAST kick over the head
    (0.3, dict(lknee=1.5, rknee=1.5, waist=0.52, **EAR), ('>=', 40, 'early')),  # 6 tuck, hands to the ears
    (0.15, dict(sp=-2.6, el=1.2), ('>=', 70, 'early')),                         # 7 push
    (0.5, dict(lknee=2.0, rknee=2.0), None),                                    # 8 land on the knees
]
S.SEQS['pump3'] = [
    (0.6, dict(EAR), None),                                                     # 1 hands beside the ears
    (1.0, legs(-1.57, 0.0), None),                                              # 2 legs up
    (0.3, dict(), None),                                                        # 3 hold
    (0.2, legs(0.4, 0.0), None),                                                # 4 slam the legs on the floor
    (0.1, dict(legs(-1.8, 0.0), waist=0.52), None),                             # 5 FAST kick UP, head forward
    (0.25, legs(-2.53, 1.0), ('>=', 30, 'early')),                              # 6 fold over the head
    (0.15, dict(sp=-2.6, el=1.2), ('>=', 70, 'early')),                         # 7 push with the hands
    (0.5, dict(lknee=2.0, rknee=2.0), None),                                    # 8 land on the knees
]

if __name__ == '__main__':
    S.main()
