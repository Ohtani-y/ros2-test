#!/usr/bin/env python3
"""Pump: legs up, slam them down on the floor (the floor pushes back), then swing up over the head.
  python3 pump.py pump      both legs
  python3 pump.py pumpR     right (heavy) leg only, left foot planted as support"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import kick as K  # noqa: E402  (registers the one-leg groups)

S = K.S
EAR = S.EAR
S.SEQS['pump'] = [
    (1.0, dict(lhip=-1.57, rhip=-1.57, lknee=0.0, rknee=0.0, sp=-2.8, el=1.5), None),  # 1 legs up, arms overhead
    (0.3, dict(), None),                                                                 # 2 hold
    (0.2, dict(lhip=0.4, rhip=0.4), None),                                               # 3 slam the legs on the floor
    (0.25, dict(lhip=-2.53, rhip=-2.53, lknee=0.5, rknee=0.5, sp=-0.6), None),          # 4 swing up, arms counter-swing
    (0.3, dict(lknee=1.5, rknee=1.5, waist=0.52, **EAR), ('>=', 40, 'early')),          # 5 tuck, hands to the ears
    (0.15, dict(sp=-2.6, el=1.2), ('>=', 70, 'early')),                                 # 6 push
    (0.5, dict(lknee=2.0, rknee=2.0), None),                                             # 7 land on the knees
]
S.SEQS['pumpR'] = [
    (1.0, dict(K.PLANT, rhip=-1.57, rknee=0.0, sp=-2.8, el=1.5), None),                 # 1 right leg up, left foot planted
    (0.3, dict(), None),
    (0.2, dict(rhip=0.4), None),                                                         # 3 slam the right leg down
    (0.25, dict(rhip=-2.53, rknee=0.5, lhip=-0.3, sp=-0.6), None),                      # 4 kick up, left foot pushes
    (0.3, dict(lhip=-2.53, lknee=1.5, rknee=1.5, waist=0.52, **EAR), ('>=', 40, 'early')),
    (0.15, dict(sp=-2.6, el=1.2), ('>=', 70, 'early')),
    (0.5, dict(lknee=2.0, rknee=2.0), None),
]

if __name__ == '__main__':
    S.main()
