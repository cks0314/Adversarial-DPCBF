"""Canonical AR-DPCBF scenario generator (DPCBF Table I params).

The n_adv obstacles closest to the robot start are the maneuvering
adversaries; the rest are benign constant-velocity movers.
"""
import numpy as np
import ardpcbf_core as C

P = C.P


def make_scn(
    seed,
    N,
    n_adv,
    GOALX=50.0,
    AOB=0.5,
    WOB=0.4,
    VNOM=1.2,
    mt=80,
):
    """Generate a deterministic scenario for a given seed."""
    kappa = AOB + VNOM * WOB
    last = None

    for tt in range(mt):
        # Use a different deterministic seed for each retry.
        rng = np.random.default_rng(seed * 1000 + tt)

        xr0 = np.array([2.0, 12.0, 0.0, P.vdes])
        goal = np.array([GOALX, 12.0])

        # Place obstacles and sample their initial speeds.

        placements = [
            (
                rng.uniform(8, GOALX - 4),
                rng.uniform(2, 22),
                rng.uniform(0.3, 1.2),
            )
            for _ in range(N)
        ]

        # The closest n_adv obstacles are adversarial.

        dist = np.array([
            np.hypot(ox - xr0[0], oy - xr0[1])
            for ox, oy, _ in placements
        ])
        adv_set = set(np.argsort(dist)[:n_adv].tolist())

        # Assign headings and build obstacle states.

        obs = []
        adv = []
        ogoals = []

        for j in range(N):
            ox, oy, ov = placements[j]

            if j in adv_set:
                # Adversaries initially point toward the robot corridor.
                oth = (
                    np.arctan2(
                        12.0 - oy,
                        (xr0[0] + 6.0) - ox,
                    )
                    + rng.uniform(-0.4, 0.4)
                )
            else:
                # Benign obstacles use random headings.
                oth = rng.uniform(-np.pi, np.pi)

            obs.append(np.array([ox, oy, oth, ov]))
            adv.append(j in adv_set)

            # Retained for compatibility with the episode code.
            ogoals.append(np.zeros(2))

        sc = {
            'xr0': xr0,
            'goal': goal,
            'obs': obs,
            'adv': adv,
            'ogoals': ogoals,
            'kappa': kappa,
            'aobs_max': AOB,
            'wobs_max': WOB,
            'GOALX': GOALX,
            'vobs_cap': 1.2,
        }
        last = sc

        # Reject initial placements that violate any nominal DPCBF.
        if all(
            C.barrier(xr0, o, kappa, False) > 0
            for o in obs
        ):
            return sc

    # Preserve the original behavior if no collision-free candidate was
    # found within mt attempts: return the final candidate.
    return last
