"""Fig: Theorem-9 validity-boundary heatmap. Requires data/boundary_nagumo.npz
(run experiments/validity_boundary.py first)."""
from _bootstrap import ROOT, DATA, FIGS
import numpy as np, matplotlib.pyplot as plt
from matplotlib.lines import Line2D
plt.rcParams.update({'font.size': 11, 'savefig.dpi': 200})
d = np.load(DATA/'boundary_nagumo.npz'); FR = d['FR']; KAP = d['KAP']; CMIN = d['CMIN']
fig, ax = plt.subplots(figsize=(8.6, 6.6))
dk = (KAP[1]-KAP[0])/2; dc = (CMIN[1]-CMIN[0])/2
kedge = np.concatenate([KAP-dk, [KAP[-1]+dk]]); cedge = np.concatenate([CMIN-dc, [CMIN[-1]+dc]])
pm = ax.pcolormesh(kedge, cedge, FR, cmap='RdYlGn', vmin=0, vmax=1, shading='flat')
cb = fig.colorbar(pm, ax=ax, fraction=0.046, pad=0.02)
cb.set_label('fraction of boundary states with $\\dot h^{*}\\geq0$\n(certificate maintainable vs worst adversary)')
KK, CC = np.meshgrid(KAP, CMIN)
ax.contour(KK, CC, FR, levels=[0.5], colors='k', linewidths=2.4)
ax.contour(KK, CC, FR, levels=[0.9], colors='#333', linewidths=1.0, linestyles=':')
kk = np.linspace(KAP.min(), KAP.max(), 50)
ax.plot(kk, kk, 'b--', lw=2.4)
ax.text(0.55, 2.55, 'Thm 9 guarantees safety here\n($c_{\\min}>\\kappa$): 100% maintainable', fontsize=9.5, color='navy')
ax.set_xlabel(r'adversary capability $\kappa$  (m s$^{-2}$)')
ax.set_ylabel(r'control authority $c_{\min}=\min\{a_{\max},\,v^2\beta_{\max}/\ell_r\}$  (m s$^{-2}$)')
ax.set_xlim(KAP.min(), KAP.max()); ax.set_ylim(CMIN.min(), CMIN.max())
ax.legend(handles=[Line2D([0],[0],color='b',ls='--',lw=2.4,label=r'Thm 9 threshold $c_{\min}=\kappa$'),
                   Line2D([0],[0],color='k',lw=2.4,label='empirical cliff (frac=0.5)')],
          loc='lower right', fontsize=9, framealpha=0.96)
ax.set_title('Validity-boundary map for Theorem 9', fontsize=12)
fig.tight_layout(); fig.savefig(FIGS/'fig_boundary.pdf'); fig.savefig(FIGS/'fig_boundary.png')
print("saved", FIGS/'fig_boundary.pdf')
