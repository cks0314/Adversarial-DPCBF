"""Fig: AR-DPCBF parabola variation with (kappa, ||v_rel||, d) in the LoS velocity frame.
Subtractive contraction from ardpcbf_core: {h*=0} (solid) lies right of {h=0} (dashed)."""
from _bootstrap import ROOT, DATA, FIGS
import numpy as np, matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import ardpcbf_core as C
P = C.P
KLAM, KMU, GAMMA, AMAX = P.klam, P.kmu, P.gamma, P.amax

def params(d, vrel, kappa, adv):
    lam = KLAM*d/vrel; mu = KMU*d
    if adv and kappa > 0:
        lam = max(lam - kappa/(GAMMA*AMAX*vrel), 0.0)
        mu = max(mu - kappa*d/(GAMMA*vrel), 0.0)
    return lam, mu

def curve(d, vrel, kappa, adv, vy):
    lam, mu = params(d, vrel, kappa, adv); return -(lam*vy*vy + mu), -mu

vy = np.linspace(-3, 3, 400)
plt.rcParams.update({'font.size': 11, 'savefig.dpi': 200, 'axes.grid': True, 'grid.alpha': 0.25})
fig, ax = plt.subplots(1, 3, figsize=(15, 4.3))
XL, XR_, YB, YT = -4.5, 2.2, -3, 3
def style(a):
    a.axhline(0, color='k', lw=0.8); a.axvline(0, color='k', lw=0.8)
    a.set_xlim(XL, XR_); a.set_ylim(YB, YT); a.set_xlabel(r'$\tilde v_{\mathrm{rel},x}$')
def pc(a, d, vrel, kappa, adv, color, ls, lw=2.0, z=2):
    x, vx0 = curve(d, vrel, kappa, adv, vy); m = (x >= XL-0.5)
    a.plot(x[m], vy[m], color=color, ls=ls, lw=lw, zorder=z)
    if XL <= vx0 <= XR_: a.plot([vx0], [0], 'o', color=color, ms=6, zorder=4)
da, va = 2.0, 1.2; cols_a = ['#d62728', '#ff7f0e', '#9467bd', '#1f77b4']; kaps = [0.0, 0.5, 1.0, 2.0]
for k, c in zip(kaps, cols_a): pc(ax[0], da, va, k, k > 0, c, '--' if k == 0 else '-', 2.2)
ax[0].legend(handles=[Line2D([0],[0],color=cols_a[0],ls='--',lw=2.2,label=r'DPCBF ($\kappa=0$)'),
    Line2D([0],[0],color=cols_a[1],lw=2.2,label=r'AR-DPCBF ($\kappa=0.5$)'),
    Line2D([0],[0],color=cols_a[2],lw=2.2,label=r'AR-DPCBF ($\kappa=1$)'),
    Line2D([0],[0],color=cols_a[3],lw=2.2,label=r'AR-DPCBF ($\kappa=2$)')], fontsize=8.5, loc='upper left')
style(ax[0]); ax[0].set_ylabel(r'$\tilde v_{\mathrm{rel},y}$')
ax[0].set_title(r'(a) capability $\kappa$  ($d=2$, $\Vert v_{\mathrm{rel}}\Vert=1.2$)', fontsize=10.5)
kb, db = 1.0, 2.0; vrels = [0.6, 1.2, 2.0, 3.5]; cols_b = ['#d62728', '#ff7f0e', '#2ca02c', '#1f77b4']
for vr, c in zip(vrels, cols_b):
    pc(ax[1], db, vr, kb, False, c, '--', 1.7, z=1); pc(ax[1], db, vr, kb, True, c, '-', 2.2, z=3)
ax[1].legend(handles=[Line2D([0],[0],color=c,lw=2.2,label=r'$\Vert v_{\mathrm{rel}}\Vert=%g$'%vr) for vr,c in zip(vrels,cols_b)], fontsize=8.5, loc='upper left')
style(ax[1]); ax[1].set_title(r'(b) $\Vert v_{\mathrm{rel}}\Vert$  ($\kappa=1$, $d=2$)', fontsize=10.5)
ax[1].text(0.02, 0.02, 'solid: AR-DPCBF\ndashed: DPCBF', transform=ax[1].transAxes, fontsize=8, va='bottom')
kc, vc = 1.0, 1.2; ds = [3.0, 2.0, 1.2, 0.6]; cols_c = ['#d62728', '#ff7f0e', '#2ca02c', '#1f77b4']
for dd, col in zip(ds, cols_c):
    pc(ax[2], dd, vc, kc, False, col, '--', 1.7, z=1); pc(ax[2], dd, vc, kc, True, col, '-', 2.2, z=3)
ax[2].legend(handles=[Line2D([0],[0],color=col,lw=2.2,label=r'$d=%g$ m'%dd) for dd,col in zip(ds,cols_c)], fontsize=8.5, loc='upper left')
style(ax[2]); ax[2].set_title(r'(c) clearance $d$  ($\kappa=1$, $\Vert v_{\mathrm{rel}}\Vert=1.2$)', fontsize=10.5)
ax[2].text(0.02, 0.02, 'solid: AR-DPCBF\ndashed: DPCBF', transform=ax[2].transAxes, fontsize=8, va='bottom')
fig.suptitle('AR-DPCBF safe-set contraction: {h*=0} (solid) lies right of {h=0} (dashed)', fontsize=10.5, y=1.0)
fig.tight_layout(rect=[0, 0, 1, 0.97])
fig.savefig(FIGS/'fig_variation.pdf'); fig.savefig(FIGS/'fig_variation.png')
print("saved", FIGS/'fig_variation.pdf')
