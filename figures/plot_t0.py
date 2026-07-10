"""Fig: initial configuration (t=0) of the deterministic silent-failure scenario (seed 9)."""
from _bootstrap import ROOT, DATA, FIGS
import numpy as np, matplotlib.pyplot as plt
from matplotlib.patches import Circle
from matplotlib.lines import Line2D
import ardpcbf_core as C
from scenario import make_scn
P = C.P; r = P.r
SEED, N, NADV = 9, 10, 5
scn = make_scn(SEED, N, NADV); obs = scn['obs']; adv = scn['adv']; start = scn['xr0']; goal = scn['goal']
plt.rcParams.update({'font.size': 11, 'savefig.dpi': 200})
fig, ax = plt.subplots(figsize=(12, 5.6))
for o, a in zip(obs, adv):
    fc = '#ffd9d6' if a else '#dddddd'; ec = '#c0392b' if a else '#888888'
    ax.add_patch(Circle((o[0], o[1]), r, fc=fc, ec=ec, lw=1.6, zorder=2))
    ax.arrow(o[0], o[1], 1.2*np.cos(o[2]), 1.2*np.sin(o[2]), head_width=0.45, head_length=0.4,
             fc=ec, ec=ec, lw=1.0, length_includes_head=True, zorder=3)
th = start[2]
ax.plot([start[0]], [start[1]], marker=(3, 0, np.degrees(th)-90), ms=16, color='#1f77b4', mec='k', mew=0.8, zorder=5)
ax.plot([start[0]], [start[1]], 'ks', ms=9, zorder=4, mfc='none', mew=1.5)
ax.arrow(start[0], start[1], 1.6*np.cos(th), 1.6*np.sin(th), head_width=0.5, head_length=0.45,
         fc='#1f77b4', ec='#1f77b4', lw=1.2, length_includes_head=True, zorder=6)
ax.plot([goal[0]], [goal[1]], '*', ms=24, color='gold', mec='k', mew=0.8, zorder=5)
ax.annotate('start', (start[0], start[1]), xytext=(start[0]-1, start[1]+1.6), fontsize=10, ha='center')
ax.annotate('goal', (goal[0], goal[1]), xytext=(goal[0], goal[1]+1.8), fontsize=10, ha='center')
ax.set_xlim(-1, 53); ax.set_ylim(1, 25); ax.set_aspect('equal', adjustable='box')
ax.set_xlabel('$x$ [m]'); ax.set_ylabel('$y$ [m]'); ax.grid(alpha=0.25)
ax.set_title(r'Initial configuration $t=0$ (seed 9, $N=10$, $n_{\mathrm{adv}}=5$, $\kappa=0.98$)', fontsize=12)
ax.legend(handles=[
    Line2D([0],[0],marker='o',ls='',mfc='#ffd9d6',mec='#c0392b',mew=1.6,ms=12,label='adversarial obstacle (keep-out radius $r$)'),
    Line2D([0],[0],marker='o',ls='',mfc='#dddddd',mec='#888888',mew=1.6,ms=12,label='benign (constant-velocity) obstacle'),
    Line2D([0],[0],marker=(3,0,0),ls='',color='#1f77b4',mec='k',ms=12,label='robot (heading $\\to$ goal)'),
    Line2D([0],[0],marker='*',ls='',color='gold',mec='k',ms=16,label='goal')], fontsize=9, loc='upper center', ncol=2, framealpha=0.95)
fig.tight_layout(); fig.savefig(FIGS/'fig_t0.pdf'); fig.savefig(FIGS/'fig_t0.png')
print("saved", FIGS/'fig_t0.pdf')
