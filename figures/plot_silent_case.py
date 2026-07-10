"""Fig: deterministic silent-failure case (seed 9). Two-row timelapse
(DPCBF breaches / Buffer safe) + h-vs-clearance traces. Self-contained
(re-simulates the episode; no cached data needed)."""
from _bootstrap import ROOT, DATA, FIGS
import numpy as np, matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.patches import Circle
from matplotlib.lines import Line2D
from mpl_toolkits.axes_grid1.inset_locator import inset_axes
import ardpcbf_core as C, ardpcbf_run as R
from scenario import make_scn
P = C.P; r = P.r
scn = make_scn(9, 10, 5); goal = scn['goal']; start = scn['xr0'][:2]
def get(meth):
    m = R.run_episode(meth, scn, dt=0.06, t_max=40.0, record_trace=True); tr = m['trace']
    return (np.array(tr['t']), np.array(tr['xr']), tr['obs'], np.array(tr['h']),
            np.array(tr['hs']), np.array(tr['clear']), m)
tD, xD, oD, hD, hsD, clD, mD = get('dpcbf')
tB, xB, oB, hB, hsB, clB, mB = get('buffer')
kc = np.where(clD < 0)[0]; tbreach = tD[kc[0]]
T = [0.0, tbreach*0.45, tbreach*0.75, tbreach]
def nearest(t, arr): return int(np.argmin(np.abs(arr-t)))
plt.rcParams.update({'font.size': 9, 'savefig.dpi': 200})
fig = plt.figure(figsize=(18, 9.2))
gs = gridspec.GridSpec(3, 4, height_ratios=[1.0, 1.0, 0.85], hspace=0.32, wspace=0.12)
XL, XR_, YB, YT = -1, 52, 1, 17
def snap(ax, t, xr_all, t_all, obs_all, color, label, breach=False):
    k = nearest(t, t_all); xr = xr_all[k]; obs = obs_all[k]
    ax.plot(xr_all[:k+1, 0], xr_all[:k+1, 1], color=color, lw=2.0, zorder=3)
    rob_breach = False
    for i, o in enumerate(obs):
        d = np.hypot(o[0]-xr[0], o[1]-xr[1]); hit = d < r; isadv = scn['adv'][i]
        if hit and breach: rob_breach = True
        if hit and breach: fc, ec = '#ff6961', '#c0392b'
        elif isadv: fc, ec = '#ffd9d6', '#c0392b'
        else: fc, ec = '#dddddd', '#888888'
        ax.add_patch(Circle((o[0], o[1]), r, fc=fc, ec=ec, lw=1.2, alpha=0.9, zorder=1))
        ax.arrow(o[0], o[1], 0.9*np.cos(o[2]), 0.9*np.sin(o[2]), head_width=0.35, head_length=0.3,
                 fc=ec, ec=ec, lw=0.8, zorder=2, length_includes_head=True)
    ax.plot([xr[0]], [xr[1]], marker=(3, 0, np.degrees(xr[2])-90), ms=11, color=color, mec='k', mew=0.6, zorder=5)
    ax.plot([start[0]], [start[1]], 'ks', ms=7, zorder=4)
    ax.plot([goal[0]], [goal[1]], '*', ms=15, color='gold', mec='k', mew=0.6, zorder=4)
    ax.set_xlim(XL, XR_); ax.set_ylim(YB, YT); ax.set_aspect('equal', adjustable='box')
    ax.set_title(f'{label}  $t={t:.1f}$ s'+('   COLLISION' if rob_breach else ''), fontsize=9,
                 color=('#c0392b' if rob_breach else 'k')); ax.grid(alpha=0.2)
    if rob_breach:
        ax.annotate('breach: $d<r$\nwhile $h\\geq0$', xy=(xr[0], xr[1]), xytext=(xr[0]-15, YB+2.0),
                    fontsize=8.5, color='#c0392b', arrowprops=dict(arrowstyle='->', color='#c0392b'))
for j, t in enumerate(T):
    a = fig.add_subplot(gs[0, j]); snap(a, t, xD, tD, oD, '#d62728', 'DPCBF', breach=(j == len(T)-1))
    if j == 0:
        a.set_ylabel('DPCBF\n$y$ [m]', fontsize=9.5)
        a.legend(handles=[Line2D([0],[0],marker='o',ls='',mfc='#ffd9d6',mec='#c0392b',mew=1.4,ms=9,label='adversarial'),
                          Line2D([0],[0],marker='o',ls='',mfc='#dddddd',mec='#888888',mew=1.4,ms=9,label='benign')],
                 fontsize=7.5, loc='lower right', framealpha=0.95, handletextpad=0.2, borderpad=0.3)
for j, t in enumerate(T):
    b = fig.add_subplot(gs[1, j]); snap(b, t, xB, tB, oB, '#2ca02c', 'Buffer Soft AR', breach=False)
    if j == 0: b.set_ylabel('Buffer Soft AR\n$y$ [m]', fontsize=9.5)
    b.set_xlabel('$x$ [m]')
axL = fig.add_subplot(gs[2, 0:2]); axR = fig.add_subplot(gs[2, 2:4])
axL.plot(tD, hD, color='#d62728', lw=2, label=r'DPCBF barrier $h(t)$')
axL.plot(tD, clD, color='#d62728', ls=':', lw=2, label=r'true clearance $d-r$')
axL.axhline(0, color='k', lw=0.8)
axL.set_title('(DPCBF) silent failure', fontsize=9.5); axL.set_xlabel('$t$ [s]'); axL.set_ylabel('value')
axL.legend(fontsize=8, loc='upper right'); axL.grid(alpha=0.25); axL.set_ylim(-1.5, 9)
sil = (hD >= -1e-3) & (clD < 0); ks = int(np.where(sil)[0][0]) if sil.any() else int(kc[0]); tb = tD[ks]
axI = inset_axes(axL, width='42%', height='42%', loc='center left',
                 bbox_to_anchor=(0.06, -0.02, 1, 1), bbox_transform=axL.transAxes)
mwin = (tD >= tb-1.6) & (tD <= tb+0.8)
axI.plot(tD[mwin], hD[mwin], color='#d62728', lw=2); axI.plot(tD[mwin], clD[mwin], color='#d62728', ls=':', lw=2)
axI.axhline(0, color='k', lw=0.8)
axI.fill_between(tD[mwin], 0, clD[mwin], where=(clD[mwin] < 0), color='#c0392b', alpha=0.30)
axI.set_ylim(-0.12, 0.7); axI.set_xticks([]); axI.tick_params(labelsize=7)
axI.set_title(r'$h\geq0$ yet $d<r$', fontsize=8, color='#c0392b', pad=2); axI.set_facecolor('#fff6f6')
axR.plot(tB, hsB, color='#2ca02c', lw=2, label=r'AR barrier $h^{*}(t)$')
axR.plot(tB, clB, color='#2ca02c', ls=':', lw=2, label=r'true clearance $d-r$')
axR.axhline(0, color='k', lw=0.8)
axR.set_title('(Buffer Soft AR) safe', fontsize=9.5); axR.set_xlabel('$t$ [s]')
axR.legend(fontsize=8, loc='upper right'); axR.grid(alpha=0.25); axR.set_ylim(-1.5, 9)
fig.suptitle('Deterministic silent-failure case (seed 9): DPCBF satisfies its QP barrier ($h\\geq0$) yet breaches the safety radius; Buffer Soft AR-DPCBF stays safe', fontsize=10.5, y=0.995)
fig.savefig(FIGS/'fig_silent_case.pdf', bbox_inches='tight'); fig.savefig(FIGS/'fig_silent_case.png', bbox_inches='tight')
print(f"DPCBF coll={mD['collided']} minclr={mD['min_clear']:+.2f} | Buffer coll={mB['collided']} reach={mB['reached']} minclr={mB['min_clear']:+.2f}")
print("saved", FIGS/'fig_silent_case.pdf')
