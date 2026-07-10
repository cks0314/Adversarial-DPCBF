"""Experiment: capability, density, and gamma sweeps (corrected nearest-adversary
scenario). Writes data/{capability,density,gamma}_sweep.npz.
Usage:  python experiments/run_core_sweeps.py [capability|density|gamma|all] [n_seeds]"""
from _bootstrap import ROOT, DATA
import numpy as np, sys, time
import ardpcbf_core as C, ardpcbf_run as R
from scenario import make_scn
P = C.P; METHODS = ['dpcbf', 'hard', 'soft', 'buffer']; DT, TMAX = 0.06, 40.0

def episode(m, sc):
    z = R.run_episode(m, sc, dt=DT, t_max=TMAX)
    return z['collided'], (z['min_h'] < -1e-3), z['infeas_rate'], z['reached']

def capability(n=40, N=10, nadv=5, kappas=(0.0,0.2,0.4,0.6,0.8,1.0,1.2,1.4)):
    data = {'kappas': np.array(kappas)}
    for kap in kappas:
        f = kap/0.98 if kap > 0 else 0.0
        coll = np.zeros((n,4),bool); viol = np.zeros((n,4),bool)
        for s in range(n):
            sc = make_scn(s, N, nadv, AOB=0.5*f, WOB=0.4*f)
            for mi,m in enumerate(METHODS):
                c,v,_,_ = episode(m, sc); coll[s,mi]=c; viol[s,mi]=v
        data[f'coll_{kap:.2f}']=coll; data[f'viol_{kap:.2f}']=viol
        print(f"kappa={kap:.2f} viol%[{' '.join(f'{100*viol[:,i].mean():3.0f}' for i in range(4))}]"); sys.stdout.flush()
    np.savez(DATA/'capability_sweep.npz', **data); print("saved capability_sweep.npz")

def density(n=20, Ns=(1,3,6,10,15,20,25,30)):
    data = {'Ns': np.array(Ns)}
    for N in Ns:
        nadv = max(1, N//2)
        infe = np.zeros((n,4)); succ = np.zeros((n,4),bool)
        for s in range(n):
            sc = make_scn(s, N, nadv)
            for mi,m in enumerate(METHODS):
                c,v,inf,re = episode(m, sc); infe[s,mi]=inf; succ[s,mi]=(re and not c)
        data[f'infeas_{N}']=infe; data[f'succ_{N}']=succ
        print(f"N={N} infeas%[{' '.join(f'{100*infe[:,i].mean():4.1f}' for i in range(4))}] succ%[{' '.join(f'{100*succ[:,i].mean():3.0f}' for i in range(4))}]"); sys.stdout.flush()
    np.savez(DATA/'density_sweep.npz', **data); print("saved density_sweep.npz")

def gamma(n=30, gammas=(0.5,1.0,2.0,3.0,5.0,10.0,20.0,50.0)):
    data = {'gammas': np.array(gammas)}
    g0 = P.gamma
    for cond,(N,nadv) in [('N1',(1,1)),('N10',(10,5))]:
        for g in gammas:
            P.gamma = g
            viol = np.zeros((n,4),bool); infe = np.zeros((n,4))
            for s in range(n):
                sc = make_scn(s, N, nadv)
                for mi,m in enumerate(METHODS):
                    c,v,inf,_ = episode(m, sc); viol[s,mi]=v; infe[s,mi]=inf
            data[f'{cond}_viol_{g:.1f}']=viol; data[f'{cond}_infeas_{g:.1f}']=infe
        print(f"{cond} done"); sys.stdout.flush()
    P.gamma = g0
    np.savez(DATA/'gamma_sweep.npz', **data); print("saved gamma_sweep.npz")

if __name__ == '__main__':
    which = sys.argv[1] if len(sys.argv) > 1 else 'all'
    n = int(sys.argv[2]) if len(sys.argv) > 2 else None
    if which in ('capability','all'): capability(**({'n':n} if n else {}))
    if which in ('density','all'):    density(**({'n':n} if n else {}))
    if which in ('gamma','all'):      gamma(**({'n':n} if n else {}))
