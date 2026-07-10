"""Experiment: validity-boundary map for Theorem 9 (Nagumo condition).
At sampled boundary states {h*=0}, test whether the worst-case Nagumo margin
sup_u inf_F hdot* >= 0, with both authority channels limited to c_min (via amax and v).
Writes data/boundary_nagumo.npz consumed by figures/plot_boundary.py."""
from _bootstrap import ROOT, DATA, FIGS
import numpy as np
import ardpcbf_core as C
P = C.P; rng = np.random.default_rng(2)

def worst_adv(xr, xo, kappa, aob, wob, eps=1e-6):
    dtho = (C.barrier(xr, xo+np.array([0,0,eps,0]), kappa, True) -
            C.barrier(xr, xo+np.array([0,0,-eps,0]), kappa, True))/(2*eps)
    dvo = (C.barrier(xr, xo+np.array([0,0,0,eps]), kappa, True) -
           C.barrier(xr, xo+np.array([0,0,0,-eps]), kappa, True))/(2*eps)
    return np.array([-aob*np.sign(dvo) if abs(dvo) > 1e-12 else 0.0,
                     -wob*np.sign(dtho) if abs(dtho) > 1e-12 else 0.0])

def hdot_star(xr, xo, kappa, aob, wob, dt=5e-4):
    Lg, _ = C.lie_Lg(xr, xo, kappa, True)
    ust = np.array([P.amax*np.sign(Lg[0]) if abs(Lg[0]) > 1e-12 else 0.0,
                    P.bmax*np.sign(Lg[1]) if abs(Lg[1]) > 1e-12 else 0.0])
    aw = worst_adv(xr, xo, kappa, aob, wob)
    xr2 = xr + dt*C.robot_deriv(xr, ust); xo2 = xo + dt*C.obs_deriv(xo, aw)
    return (C.barrier(xr2, xo2, kappa, True) - C.barrier(xr, xo, kappa, True))/dt

def cell(kappa, cm, Ns=300, maxtry=120000):
    aob = kappa/2.0; wob = (kappa/2.0)/1.2; v = np.sqrt(cm/(P.bmax/P.lr))
    a0 = P.amax; P.amax = cm            # both authority channels limited to c_min
    hd = []; t = 0
    while len(hd) < Ns and t < maxtry:
        t += 1
        phi = rng.uniform(0, 2*np.pi); rho = rng.uniform(1.6, 9.0)
        thr = rng.uniform(0, 2*np.pi); tho = rng.uniform(0, 2*np.pi); vo = rng.uniform(0, 1.2)
        xr = np.array([0.0, 0.0, thr, v]); xo = np.array([rho*np.cos(phi), rho*np.sin(phi), tho, vo])
        if abs(C.barrier(xr, xo, kappa, True)) > 0.06: continue
        hd.append(hdot_star(xr, xo, kappa, aob, wob))
    P.amax = a0
    hd = np.array(hd) if hd else np.array([np.nan])
    return np.mean(hd >= -1e-4), np.percentile(hd, 5)

if __name__ == '__main__':
    KAP = np.linspace(0.2, 2.6, 13); CMIN = np.linspace(0.3, 3.0, 12)
    FR = np.full((len(CMIN), len(KAP)), np.nan); WC = np.full((len(CMIN), len(KAP)), np.nan)
    for i, cm in enumerate(CMIN):
        for j, kp in enumerate(KAP):
            FR[i, j], WC[i, j] = cell(kp, cm)
        print(f"cmin={cm:.2f} frac_ok:[{' '.join(f'{FR[i,jj]:.2f}' for jj in range(len(KAP)))}]")
    np.savez(DATA/'boundary_nagumo.npz', FR=FR, WC=WC, KAP=KAP, CMIN=CMIN)
    print("saved", DATA/'boundary_nagumo.npz')
