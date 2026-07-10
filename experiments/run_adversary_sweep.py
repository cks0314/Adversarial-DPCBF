"""Experiment: adversary-count sweep at N=15 (violation + collision, per-trial).
Corrected nearest-adversary scenario, kappa=0.98. Writes data/adversary_sweep.npz.
Usage:  python experiments/run_adversary_sweep.py [n_seeds] [nadv levels...]
Default runs all levels; heavy conditions are slow (robot may not reach), so this
appends to the .npz per level and can be resumed one level at a time."""
from _bootstrap import ROOT, DATA
import numpy as np, time, sys, os
import ardpcbf_run as R
from scenario import make_scn
METHODS = ['dpcbf', 'hard', 'soft', 'buffer']
DT, TMAX = 0.06, 40.0
MASTER = DATA/'adversary_sweep.npz'

def run(n=40, levels=(0,1,3,5,7,9,11,13,15), N=15):
    data = dict(np.load(MASTER)) if os.path.exists(MASTER) else {}
    for na in levels:
        t0 = time.time()
        coll = np.zeros((n, 4), bool); viol = np.zeros((n, 4), bool)
        for s in range(n):
            sc = make_scn(s, N, na)
            for mi, m in enumerate(METHODS):
                mt = R.run_episode(m, sc, dt=DT, t_max=TMAX)
                coll[s, mi] = mt['collided']; viol[s, mi] = (mt['min_h'] < -1e-3)
        data[f'coll{na}'] = coll; data[f'viol{na}'] = viol
        np.savez(MASTER, **data)
        print(f"nadv={na:2d} viol%[{' '.join(f'{100*viol[:,i].mean():3.0f}' for i in range(4))}] "
              f"coll%[{' '.join(f'{100*coll[:,i].mean():3.0f}' for i in range(4))}] ({time.time()-t0:.0f}s)")
        sys.stdout.flush()

if __name__ == '__main__':
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 40
    levels = [int(x) for x in sys.argv[2:]] if len(sys.argv) > 2 else (0,1,3,5,7,9,11,13,15)
    run(n, levels)
