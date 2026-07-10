"""Episode runner and scenario generator for the AR-DPCBF study."""
import numpy as np
import ardpcbf_core as C
P = C.P

def make_scenario(seed, N, n_adv, aobs_max, wobs_max, vobs_nom=1.2):
    """
    Robot goes from (0,0) to (GOALX,0). N obstacles in a band ahead; n_adv of them
    are adversarial (spawned to intercept, heading toward the robot path); the rest
    are benign goal-directed movers at constant velocity.
    Returns dict with robot start/goal, obstacle states, adversarial mask, capability.
    """
    rng = np.random.default_rng(seed)
    GOALX = 24.0
    xr0 = np.array([0.0, 0.0, 0.0, P.vdes])
    goal = np.array([GOALX, 0.0])
    obs = []; adv = []; ogoals = []
    idx = rng.permutation(N)
    adv_set = set(idx[:n_adv].tolist())
    for j in range(N):
        # spawn along the corridor ahead of the robot
        ox = rng.uniform(6.0, GOALX-3.0)
        oy = rng.uniform(-5.0, 5.0)
        if j in adv_set:
            # heading roughly toward the corridor centre / robot path, modest speed
            oth = np.arctan2(0.0-oy, 2.0-ox) + rng.uniform(-0.3,0.3)
            ov  = rng.uniform(0.6, vobs_nom)
            ogoal = np.array([0.0, 0.0])  # unused for adversary
        else:
            # benign: crossing the corridor at constant velocity toward a far goal
            gy = rng.uniform(-6,6)*np.sign(rng.uniform(-1,1) if oy==0 else -oy)
            oth = rng.uniform(-np.pi, np.pi)
            ov  = rng.uniform(0.4, vobs_nom)
            ogoal = np.array([ox + np.cos(oth)*20, oy + np.sin(oth)*20])
        obs.append(np.array([ox, oy, oth, ov]))
        adv.append(j in adv_set)
        ogoals.append(ogoal)
    kappa = aobs_max + vobs_nom*wobs_max
    return dict(xr0=xr0, goal=goal, obs=obs, adv=adv, ogoals=ogoals,
                kappa=kappa, aobs_max=aobs_max, wobs_max=wobs_max, GOALX=GOALX)

def run_episode(method, scn, dt=0.05, t_max=18.0, record_trace=False):
    """Simulate one episode. Returns a metrics dict (and optional traces)."""
    xr = scn['xr0'].copy()
    obs = [o.copy() for o in scn['obs']]
    adv = scn['adv']; ogoals = scn['ogoals']
    kappa = scn['kappa']; aob = scn['aobs_max']; wob = scn['wobs_max']
    goal = scn['goal']
    nsteps = int(t_max/dt)
    min_h = np.inf; min_hs = np.inf; min_clear = np.inf
    collided = False; infeas = 0; path = 0.0; reached = False; t_goal = t_max
    tr = {'t':[], 'h':[], 'hs':[], 'clear':[], 'xr':[], 'obs':[]}
    for k in range(nsteps):
        # robot control
        u, feas = C.compute_control(method, xr, obs, goal, kappa)
        if not feas: infeas += 1
        xr_prev = xr.copy()
        xr = C.robot_step(xr, u, dt)
        path += np.hypot(xr[0]-xr_prev[0], xr[1]-xr_prev[1])
        # obstacles step
        for j in range(len(obs)):
            if adv[j]:
                aw = C.adversary_input(xr, obs[j], kappa, aob, wob)
            else:
                # benign: gentle heading hold toward goal, constant speed
                thg = np.arctan2(ogoals[j][1]-obs[j][1], ogoals[j][0]-obs[j][0])
                we = np.clip(2.0*np.arctan2(np.sin(thg-obs[j][2]), np.cos(thg-obs[j][2])), -wob, wob)
                aw = np.array([0.0, 0.0])  # constant velocity (no accel, no turn) -> DPCBF-valid
            obs[j] = C.obs_step(obs[j], aw, dt)
            if obs[j][3] > scn.get('vobs_cap', 2.0):
                obs[j][3] = scn.get('vobs_cap', 2.0)
        # metrics over obstacles
        hmins = np.inf; hsmins = np.inf; cmins = np.inf
        for o in obs:
            nP = np.hypot(o[0]-xr[0], o[1]-xr[1])
            cmins = min(cmins, nP - P.r)
            if nP - P.r < 0: collided = True
            if nP <= P.sense + 1.0:
                hmins  = min(hmins,  C.barrier(xr, o, kappa, False))
                hsmins = min(hsmins, C.barrier(xr, o, kappa, True))
        min_h = min(min_h, hmins); min_hs = min(min_hs, hsmins); min_clear = min(min_clear, cmins)
        if record_trace:
            tr['t'].append(k*dt); tr['h'].append(hmins); tr['hs'].append(hsmins)
            tr['clear'].append(cmins); tr['xr'].append(xr.copy())
            tr['obs'].append([o.copy() for o in obs])
        if not reached and (goal[0]-xr[0]) < 0.6:
            reached = True; t_goal = k*dt
            break
    m = dict(collided=collided, viol_h=(min_h<0), viol_hs=(min_hs<0),
             min_h=min_h, min_hs=min_hs, min_clear=min_clear,
             infeas=infeas, infeas_rate=infeas/nsteps, path=path,
             reached=reached, t_goal=t_goal)
    if record_trace:
        m['trace'] = tr
    return m

if __name__ == '__main__':
    import time
    scn = make_scenario(0, N=8, n_adv=4, aobs_max=1.0, wobs_max=0.8)
    t0=time.time()
    for meth in ['dpcbf','hard','soft','buffer']:
        m = run_episode(meth, scn)
        print(f"{meth:7s} collide={m['collided']!s:5s} viol_h*={m['viol_hs']!s:5s} "
              f"min_h*={m['min_hs']:+.3f} min_clear={m['min_clear']:+.3f} "
              f"infeas={m['infeas']:4d} path={m['path']:.1f} reached={m['reached']!s:5s} tg={m['t_goal']:.1f}")
    print("4 episodes in %.2fs"%(time.time()-t0))
