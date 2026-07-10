"""Canonical AR-DPCBF scenario generator (DPCBF Table I params).

Adversary definition (corrected): the ``n_adv`` obstacles CLOSEST to the robot
start are the maneuvering adversaries; all others are benign goal-directed
movers at constant velocity. A feasibility retry loop guarantees the start is
collision-free (all barriers positive at t=0).
"""
import numpy as np
import ardpcbf_core as C
P = C.P


def make_scn(seed, N, n_adv, GOALX=50.0, AOB=0.5, WOB=0.4, VNOM=1.2, mt=80):
    """Deterministic scenario for a given integer ``seed``.

    Returns a dict: xr0, goal, obs (list of [x,y,theta,v]), adv (bool mask),
    ogoals, kappa, aobs_max, wobs_max, GOALX, vobs_cap.
    """
    k = AOB + VNOM * WOB
    last = None
    for tt in range(mt):
        rng = np.random.default_rng(seed * 1000 + tt)
        xr0 = np.array([2., 12., 0., P.vdes]); goal = np.array([GOALX, 12.])
        # 1) place all obstacle positions + speeds
        P_ = [(rng.uniform(8, GOALX - 4), rng.uniform(2, 22), rng.uniform(0.3, 1.2))
              for _ in range(N)]
        # 2) adversarial := n_adv closest to robot start (Euclidean)
        dist = np.array([np.hypot(ox - xr0[0], oy - xr0[1]) for (ox, oy, ov) in P_])
        aset = set(np.argsort(dist)[:n_adv].tolist())
        # 3) headings: adversaries home toward the robot corridor; benign random
        obs = []; adv = []; og = []
        for j in range(N):
            ox, oy, ov = P_[j]
            if j in aset:
                oth = np.arctan2(12.0 - oy, (xr0[0] + 6.0) - ox) + rng.uniform(-0.4, 0.4)
            else:
                oth = rng.uniform(-np.pi, np.pi)
            obs.append(np.array([ox, oy, oth, ov])); adv.append(j in aset); og.append(np.zeros(2))
        sc = dict(xr0=xr0, goal=goal, obs=obs, adv=adv, ogoals=og, kappa=k,
                  aobs_max=AOB, wobs_max=WOB, GOALX=GOALX, vobs_cap=1.2)
        last = sc
        if all(C.barrier(xr0, o, k, False) > 0 for o in obs):
            return sc
    return last
