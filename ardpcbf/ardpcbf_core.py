"""
AR-DPCBF simulation core.
Kinematic bicycle robot vs maneuvering (unicycle) obstacles.
Barriers and controllers follow the manuscript (subtractive/contracting convention).

Conventions (verified against the manuscript):
  Robot state  xr = [x, y, theta, v]            (kinematic bicycle, eq (9))
  Obstacle st. xo = [x, y, theta, v]            (unicycle, eq (16))
  Robot dyn:   xr_dot = f(xr) + g(xr) u,  u = [a, beta],  |a|<=amax, |beta|<=bmax
               f = [v cos th, v sin th, 0, 0]
               g = [[0,-v sin th],[0, v cos th],[0, v/lr],[1,0]]
               (=> position has slip:  p_rob_dot = v e_rob + v beta e_rob_perp)
  LoS barrier: h  = vtx + lam  vty^2 + mu      (DPCBF, eq (14))
               h* = vtx + lam* vty^2 + mu*     (AR-DPCBF, contracted)
               lam = klam d/||vrel||, mu = kmu d
               lam* = lam - kappa/(gamma amax ||vrel||),  mu* = mu - kappa d/(gamma ||vrel||)
               kappa = aobs_max + vobs_max*wobs_max
  Safe set     C* = {h* >= 0}  (subset of {h>=0}: contraction => safety inheritance)
"""
import numpy as np

# ----------------------------- parameters -----------------------------
class P:
    # --- kinematic-bicycle / DPCBF Table I parameters (matched to Park et al.) ---
    lr     = 0.20          # rear-axle distance
    klam   = 0.144
    kmu    = 0.505
    gamma  = 3.0           # AR pre-emption gain (buffer reserves 1/gamma of worst manoeuvre)
    amax   = 5.0           # robot longitudinal accel limit (also the reference accel in Dlam)
    bmax   = 0.28          # robot slip-angle limit (beta_max)
    vmin   = 1.0           # min speed: steering authority ~ v^2/lr vanishes at rest, so
                           #            c_min = min(amax, vmin^2*bmax/lr) = 1.40 here
    vmax   = 3.5
    vdes   = 2.5           # reference cruise speed
    alpha0 = 2.0           # class-K: alpha(h)=alpha0*h
    r      = 1.0           # combined safety radius
    sense  = 15.0          # sensing range: obstacles beyond this impose no constraint
    # soft-penalty params
    rho    = 10.0
    eps_b  = 0.3           # buffer width (Huber)
    # reference go-to-goal gains
    Kv     = 1.5
    Kth    = 2.0

# ----------------------------- dynamics --------------------------------
def robot_deriv(xr, u):
    x, y, th, v = xr
    a, be = u
    c, s = np.cos(th), np.sin(th)
    return np.array([v*c - v*be*s,
                     v*s + v*be*c,
                     (v/P.lr)*be,
                     a])

def robot_step(xr, u, dt):
    # RK4
    k1 = robot_deriv(xr, u)
    k2 = robot_deriv(xr + 0.5*dt*k1, u)
    k3 = robot_deriv(xr + 0.5*dt*k2, u)
    k4 = robot_deriv(xr + dt*k3, u)
    xr = xr + (dt/6.0)*(k1+2*k2+2*k3+k4)
    xr[3] = np.clip(xr[3], P.vmin, P.vmax)   # speed limits
    return xr

def obs_deriv(xo, aw):
    x, y, th, v = xo
    a, w = aw
    return np.array([v*np.cos(th), v*np.sin(th), w, a])

def obs_step(xo, aw, dt):
    k1 = obs_deriv(xo, aw)
    k2 = obs_deriv(xo + 0.5*dt*k1, aw)
    k3 = obs_deriv(xo + 0.5*dt*k2, aw)
    k4 = obs_deriv(xo + dt*k3, aw)
    xo = xo + (dt/6.0)*(k1+2*k2+2*k3+k4)
    xo[3] = max(xo[3], 0.0)
    return xo

# ----------------------------- barrier ---------------------------------
def barrier(xr, xo, kappa, adversarial):
    """Return h (or h*) value. adversarial=False -> DPCBF h; True -> AR-DPCBF h*."""
    p_rel = xo[:2] - xr[:2]
    nP = np.hypot(p_rel[0], p_rel[1])
    nP = max(nP, 1e-6)
    vrob = xr[3]*np.array([np.cos(xr[2]), np.sin(xr[2])])      # no-slip robot velocity
    vobs = xo[3]*np.array([np.cos(xo[2]), np.sin(xo[2])])
    vrel = vobs - vrob
    xhat = p_rel/nP
    yhat = np.array([-xhat[1], xhat[0]])
    vtx = vrel @ xhat
    vty = vrel @ yhat
    d  = np.sqrt(max(nP*nP - P.r*P.r, 1e-9))
    nv = max(np.hypot(vrel[0], vrel[1]), 0.2)   # engagement floor on ||v_rel||
    lam = P.klam*d/nv
    mu  = P.kmu*d
    if adversarial and kappa > 0.0:
        # subtract the manoeuvre buffer; clamp at 0 (Thm 2(iii): the parabola
        # cannot invert -- past the validity floor we use the most-contracted
        # certificate of this form, lam*=mu*=0)
        lam = max(lam - kappa/(P.gamma*P.amax*nv), 0.0)
        mu  = max(mu  - kappa*d/(P.gamma*nv),       0.0)
    return vtx + lam*vty*vty + mu

def _part(xr, xo, kappa, adv, which, eps=1e-6):
    """Central-difference partials of the barrier. which: list of (frame,i)
    with frame in {'r','o'} and i the state index. Returns dict."""
    out = {}
    for fr, i in which:
        if fr == 'r':
            dr = np.zeros(4); dr[i] = eps
            out[(fr,i)] = (barrier(xr+dr,xo,kappa,adv)-barrier(xr-dr,xo,kappa,adv))/(2*eps)
        else:
            do = np.zeros(4); do[i] = eps
            out[(fr,i)] = (barrier(xr,xo+do,kappa,adv)-barrier(xr,xo-do,kappa,adv))/(2*eps)
    return out

def lie(xr, xo, kappa, adversarial):
    """(Lf, Lg[2], h) for the CBF-QP, constant-velocity obstacle. 12 barrier evals."""
    x, y, th, v = xr
    c, s = np.cos(th), np.sin(th)
    g = _part(xr, xo, kappa, adversarial,
              [('r',0),('r',1),('r',2),('r',3),('o',0),('o',1)])
    Lf = g[('r',0)]*(v*c) + g[('r',1)]*(v*s) \
         + g[('o',0)]*(xo[3]*np.cos(xo[2])) + g[('o',1)]*(xo[3]*np.sin(xo[2]))
    Lg_a    = g[('r',3)]
    Lg_beta = g[('r',0)]*(-v*s) + g[('r',1)]*(v*c) + g[('r',2)]*(v/P.lr)
    h = barrier(xr, xo, kappa, adversarial)
    return Lf, np.array([Lg_a, Lg_beta]), h, None, None

def lie_Lg(xr, xo, kappa, adversarial):
    """Just Lg[2] and h for the penalty term (8 barrier evals)."""
    x, y, th, v = xr; c, s = np.cos(th), np.sin(th)
    g = _part(xr, xo, kappa, adversarial, [('r',0),('r',1),('r',2),('r',3)])
    Lg_a    = g[('r',3)]
    Lg_beta = g[('r',0)]*(-v*s) + g[('r',1)]*(v*c) + g[('r',2)]*(v/P.lr)
    return np.array([Lg_a, Lg_beta]), barrier(xr, xo, kappa, adversarial)

def grad_barrier(xr, xo, kappa, adversarial, eps=1e-6):
    """Full (g_r[4], g_o[4]) -- used only by sanity tests."""
    g = _part(xr, xo, kappa, adversarial, [(f,i) for f in 'ro' for i in range(4)], eps)
    return (np.array([g[('r',i)] for i in range(4)]),
            np.array([g[('o',i)] for i in range(4)]))

# ----------------------------- 2D projection QP ------------------------
def qp_project(uref, A, b, lo, hi):
    """
    min ||u-uref||^2  s.t.  A u >= b  (rows),  lo<=u<=hi.
    2D exact projection by KKT/vertex enumeration. Returns (u, feasible).
    """
    uref = np.asarray(uref, float)
    # assemble all halfplane constraints A_all u >= b_all  (incl. box)
    Aall = [np.array([1.0,0.0]), np.array([-1.0,0.0]),
            np.array([0.0,1.0]), np.array([0.0,-1.0])]
    ball = [lo[0], -hi[0], lo[1], -hi[1]]
    for i in range(len(b)):
        Aall.append(np.asarray(A[i], float)); ball.append(float(b[i]))
    Aall = np.array(Aall); ball = np.array(ball)
    tol = 1e-7
    def feasible(u):
        return np.all(Aall @ u >= ball - tol)
    cands = []
    if feasible(uref):
        return uref.copy(), True
    # single-constraint projections
    for i in range(len(ball)):
        ai = Aall[i]; nrm2 = ai@ai
        if nrm2 < 1e-12: continue
        u = uref + (ball[i] - ai@uref)/nrm2 * ai
        cands.append(u)
    # pairwise vertices
    n = len(ball)
    for i in range(n):
        for j in range(i+1, n):
            M = np.array([Aall[i], Aall[j]])
            det = np.linalg.det(M)
            if abs(det) < 1e-9: continue
            u = np.linalg.solve(M, np.array([ball[i], ball[j]]))
            cands.append(u)
    best=None; bestd=np.inf
    for u in cands:
        if feasible(u):
            dd = np.sum((u-uref)**2)
            if dd < bestd: bestd=dd; best=u
    if best is None:
        # infeasible: return the least-infeasible box point near uref (clamp), flag False
        u = np.array([np.clip(uref[0], lo[0], hi[0]), np.clip(uref[1], lo[1], hi[1])])
        return u, False
    return best, True

# ----------------------------- controllers -----------------------------
def reference_control(xr, goal):
    """Go-to-goal reference (a, beta)."""
    dx = goal[0]-xr[0]; dy = goal[1]-xr[1]
    th_goal = np.arctan2(dy, dx)
    err = np.arctan2(np.sin(th_goal-xr[2]), np.cos(th_goal-xr[2]))
    a   = P.Kv*(P.vdes - xr[3])
    be  = P.Kth*err
    return np.array([np.clip(a, -P.amax, P.amax), np.clip(be, -P.bmax, P.bmax)])

def dphi_soft(s):
    # phi=max(0,-s)^2 ; dphi/ds = 2 min(0,s)
    return 2.0*min(0.0, s)

def dphi_buffer(s, eps_b):
    # Huber buffer (eq 70)
    if s > eps_b:   return 0.0
    if s > 0.0:     return -(eps_b - s)/eps_b
    return -1.0

def compute_control(method, xr, obs_list, goal, kappa):
    """
    method in {'dpcbf','hard','soft','buffer'}.
    Returns (u, feasible_flag). For 'hard', feasible_flag=False means the AR-QP
    was infeasible and the controller fell back to the DPCBF QP.
    """
    uref = reference_control(xr, goal)
    lo = np.array([-P.amax, -P.bmax]); hi = np.array([P.amax, P.bmax])
    near = [xo for xo in obs_list
            if np.hypot(xo[0]-xr[0], xo[1]-xr[1]) <= P.sense]

    def dpcbf_constraints():
        A=[]; b=[]
        for xo in near:
            Lf, Lg, h, _, _ = lie(xr, xo, kappa, adversarial=False)
            A.append(Lg); b.append(-P.alpha0*h - Lf)
        return A, b

    if method == 'dpcbf':
        A, b = dpcbf_constraints()
        u, feas = qp_project(uref, A, b, lo, hi)
        return u, feas

    if method == 'hard':
        A=[]; b=[]
        for xo in near:
            Lf, Lg, h, _, _ = lie(xr, xo, kappa, adversarial=True)
            A.append(Lg); b.append(-P.alpha0*h - Lf)
        u, feas = qp_project(uref, A, b, lo, hi)
        if not feas:                      # AR-QP infeasible -> fall back to DPCBF
            Ad, bd = dpcbf_constraints()
            u, _ = qp_project(uref, Ad, bd, lo, hi)
            return u, False               # flag the infeasibility event
        return u, True

    # soft / buffer: HARD constraint is DPCBF; adversarial barrier only penalised
    A, b = dpcbf_constraints()
    qpen = np.zeros(2)
    for xo in near:
        Lgs, hs = lie_Lg(xr, xo, kappa, adversarial=True)
        d = dphi_soft(hs) if method == 'soft' else dphi_buffer(hs, P.eps_b)
        qpen += P.rho * d * Lgs
    uref_eff = uref - 0.5*qpen            # min ||u-uref||^2 + qpen^T u  (eq 73)
    u, feas = qp_project(uref_eff, A, b, lo, hi)
    return u, feas

# ----------------------------- adversary -------------------------------
def adversary_input(xr, xo, kappa, aobs_max, wobs_max):
    """
    Worst-case maneuver in F minimising the robot's DPCBF barrier rate hdot.
    hdot depends on (a_obs,w_obs) via dh/dv_o (->a_obs) and dh/dtheta_o (->w_obs);
    minimiser is a box corner. 4 barrier evals.
    """
    g = _part(xr, xo, kappa, False, [('o',2),('o',3)])   # dh/dtheta_o, dh/dv_o
    dh_dtho, dh_dvo = g[('o',2)], g[('o',3)]
    a = -aobs_max*np.sign(dh_dvo)  if abs(dh_dvo)  > 1e-12 else 0.0
    w = -wobs_max*np.sign(dh_dtho) if abs(dh_dtho) > 1e-12 else 0.0
    return np.array([a, w])
