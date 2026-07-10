"""Fig: capability, density, and gamma sweeps. Reads data/{capability,density,gamma}_sweep.npz
(run experiments/run_core_sweeps.py first). Generates results/fig_{capability,density,gamma}.*"""
from _bootstrap import ROOT, DATA, FIGS
import numpy as np, matplotlib.pyplot as plt
plt.rcParams.update({'font.size': 11, 'savefig.dpi': 200})
NAMES = ['DPCBF', 'Hard AR', 'Soft AR', 'Buffer Soft AR']
COLS = ['#ff7f0e', '#1f77b4', '#2ca02c', '#9467bd']; MK = ['o', 's', 'D', '^']
def boot(x, B=4000):
    x = x.astype(float); m = x.mean()*100
    bs = np.array([np.random.choice(x, len(x)).mean() for _ in range(B)])*100
    return m, m-np.percentile(bs, 2.5), np.percentile(bs, 97.5)-m

def capability():
    d = dict(np.load(DATA/'capability_sweep.npz')); ks = d['kappas']
    fig, ax = plt.subplots(figsize=(7, 5))
    for i in range(4):
        st = [boot(d[f'viol_{k:.2f}'][:, i]) for k in ks]
        ax.errorbar(ks, [s[0] for s in st], yerr=[[s[1] for s in st], [s[2] for s in st]],
                    marker=MK[i], ms=6, capsize=3, lw=2, color=COLS[i], label=NAMES[i])
    ax.axvline(1.40, color='gray', ls=':', lw=1.5); ax.text(1.41, 5, r'$\kappa=c_{\min}$', fontsize=9)
    ax.set_xlabel(r'adversary capability $\kappa$ (m s$^{-2}$)'); ax.set_ylabel('barrier-violation rate (%)')
    ax.set_title(r'Capability sweep ($N=10$, $n_{\mathrm{adv}}=5$)'); ax.set_ylim(-3, 103)
    ax.grid(alpha=0.25); ax.legend(fontsize=9); fig.tight_layout()
    fig.savefig(FIGS/'fig_capability.pdf'); fig.savefig(FIGS/'fig_capability.png'); print('saved fig_capability')

def density():
    d = dict(np.load(DATA/'density_sweep.npz')); Ns = d['Ns']
    fig, ax = plt.subplots(1, 2, figsize=(13, 5))
    for i in range(4):
        inf = [100*d[f'infeas_{N}'][:, i].mean() for N in Ns]
        ax[0].plot(Ns, inf, marker=MK[i], lw=2, color=COLS[i], label=NAMES[i])
        st = [boot(d[f'succ_{N}'][:, i]) for N in Ns]
        ax[1].errorbar(Ns, [s[0] for s in st], yerr=[[s[1] for s in st], [s[2] for s in st]],
                       marker=MK[i], ms=5, capsize=3, lw=2, color=COLS[i], label=NAMES[i])
    ax[0].set_xlabel('obstacle count $N$'); ax[0].set_ylabel('QP-infeasibility rate (%)')
    ax[0].set_title('(a) Feasibility (Props 11-12)'); ax[0].grid(alpha=0.25); ax[0].legend(fontsize=9)
    ax[1].set_xlabel('obstacle count $N$'); ax[1].set_ylabel('success rate (%)')
    ax[1].set_title('(b) Liveness'); ax[1].set_ylim(-3, 103); ax[1].grid(alpha=0.25); ax[1].legend(fontsize=9)
    fig.tight_layout(); fig.savefig(FIGS/'fig_density.pdf'); fig.savefig(FIGS/'fig_density.png'); print('saved fig_density')

def gamma():
    d = dict(np.load(DATA/'gamma_sweep.npz')); gs = d['gammas']
    fig, ax = plt.subplots(1, 2, figsize=(13, 5))
    for pi, cond in enumerate(['N1', 'N10']):
        for i in range(4):
            v = [100*d[f'{cond}_viol_{g:.1f}'][:, i].mean() for g in gs]
            ax[pi].plot(gs, v, marker=MK[i], lw=2, color=COLS[i], label=NAMES[i])
        ax[pi].set_xscale('log'); ax[pi].set_xlabel(r'pre-emption gain $\gamma$')
        ax[pi].set_ylabel('violation rate (%)'); ax[pi].set_ylim(-3, 103); ax[pi].grid(alpha=0.25)
        ax[pi].set_title(f'({"a" if pi==0 else "b"}) {"N=1" if cond=="N1" else "N=10"}'); ax[pi].legend(fontsize=9)
    fig.tight_layout(); fig.savefig(FIGS/'fig_gamma.pdf'); fig.savefig(FIGS/'fig_gamma.png'); print('saved fig_gamma')

if __name__ == '__main__':
    import sys
    which = sys.argv[1] if len(sys.argv) > 1 else 'all'
    if which in ('capability', 'all'):
        try: capability()
        except FileNotFoundError: print('run: experiments/run_core_sweeps.py capability')
    if which in ('density', 'all'):
        try: density()
        except FileNotFoundError: print('run: experiments/run_core_sweeps.py density')
    if which in ('gamma', 'all'):
        try: gamma()
        except FileNotFoundError: print('run: experiments/run_core_sweeps.py gamma')
