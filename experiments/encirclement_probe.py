"""Probe: symmetric all-sides encirclement (negative result, kept for the record).
Robot at centre, N adversaries on a ring homing inward, goal outside. Sweeps radius
and capability. Documents that the symmetric ring does NOT separate DPCBF from the
soft AR variants: below a threshold all methods escape; above it all collide (and the
conservative variants can be trapped and do worse). See docs/notes_encirclement.md."""
from _bootstrap import ROOT, DATA
import numpy as np, sys
import ardpcbf_core as C, ardpcbf_run as R
P = C.P; CX, CY = 10.0, 8.0

def make_ring(N, Rr, aob, wob, vnom, goalx=16.0, phase=np.pi/10):
    xr0 = np.array([CX, CY, 0.0, P.vdes]); goal = np.array([CX+goalx, CY])
    obs = []; adv = []; og = []
    for i in range(N):
        th = 2*np.pi*i/N + phase; ox, oy = CX+Rr*np.cos(th), CY+Rr*np.sin(th)
        obs.append(np.array([ox, oy, np.arctan2(CY-oy, CX-ox), vnom])); adv.append(True); og.append(np.array([CX, CY]))
    return dict(xr0=xr0, goal=goal, obs=obs, adv=adv, ogoals=og, kappa=aob+vnom*wob,
                aobs_max=aob, wobs_max=wob, GOALX=CX+goalx, vobs_cap=1.0)

if __name__ == '__main__':
    for Rr in [6.0, 8.0, 9.0]:
        for (aob, wob, vnom) in [(0.25, 0.2, 0.3), (0.5, 0.4, 0.6), (0.8, 0.6, 1.2)]:
            scn = make_ring(10, Rr, aob, wob, vnom)
            row = f"R={Rr} kap={scn['kappa']:.2f} |"
            for m in ['dpcbf', 'hard', 'soft', 'buffer']:
                mm = R.run_episode(m, scn, dt=0.05, t_max=24.0)
                tag = 'COLL' if mm['collided'] else 'safe'
                row += f" {m}:{tag} cl{mm['min_clear']:+.2f} re{int(mm['reached'])} |"
            print(row)
