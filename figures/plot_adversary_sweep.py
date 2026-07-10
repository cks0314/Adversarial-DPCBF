"""Fig: adversary-count sweep (N=15) with bootstrap 95% CIs. Requires
data/adversary_sweep.npz (run experiments/run_adversary_sweep.py first)."""
from _bootstrap import ROOT, DATA, FIGS
import numpy as np, matplotlib.pyplot as plt
plt.rcParams.update({'font.size': 11, 'savefig.dpi': 200})
d = dict(np.load(DATA/'adversary_sweep.npz'))
levels = sorted(int(k[4:]) for k in d if k.startswith('coll'))
names = ['DPCBF', 'AR-DPCBF (hard)', 'Soft AR-DPCBF', 'Buffer Soft AR-DPCBF']
cols = ['#ff7f0e', '#1f77b4', '#2ca02c', '#9467bd']; mk = ['o', 's', 'D', '^']
def boot(x, B=4000):
    x = x.astype(float); m = x.mean()*100
    bs = np.array([np.random.choice(x, len(x)).mean() for _ in range(B)])*100
    return m, np.percentile(bs, 2.5), np.percentile(bs, 97.5)
fig, ax = plt.subplots(1, 2, figsize=(15, 5.4))
for panel, key, ttl in [(0, 'viol', r'(a) Barrier violation rate ($\min_t h<0$)'),
                        (1, 'coll', r'(b) Collision rate ($d<r$)')]:
    a = ax[panel]
    for i in range(4):
        ms = [boot(d[f'{key}{na}'][:, i]) for na in levels]
        m = [z[0] for z in ms]; lo = [z[0]-z[1] for z in ms]; hi = [z[2]-z[0] for z in ms]
        a.errorbar(levels, m, yerr=[lo, hi], marker=mk[i], ms=6, capsize=3, lw=2, color=cols[i], label=names[i])
    a.set_xlabel(r'Number of adversarial obstacles $n_{\mathrm{adv}}$  (of $N=15$)')
    a.set_ylabel('Violation rate (%)' if panel == 0 else 'Collision rate (%)')
    a.set_title(ttl, fontsize=11); a.set_ylim(-3, 103); a.grid(alpha=0.25); a.legend(fontsize=9)
fig.suptitle('Adversary-count sweep at high density ($N=15$, $\\kappa=0.98$, bootstrap 95% CIs)', fontsize=11, y=1.0)
fig.tight_layout(); fig.savefig(FIGS/'fig_adversary_sweep.pdf'); fig.savefig(FIGS/'fig_adversary_sweep.png')
print("saved", FIGS/'fig_adversary_sweep.pdf')
