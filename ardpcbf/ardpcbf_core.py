"""AR-DPCBF simulation core.

Kinematic bicycle robot vs maneuvering (unicycle) obstacles.
Barriers and controllers follow the manuscript (subtractive/contracting convention).
"""
import numpy as np


# ----------------------------- parameters -----------------------------
class P:

    # --- kinematic-bicycle / DPCBF parameters ---
    lr = 0.20          # Distance from rear axle to the reference point [m].
    klam = 0.144
    kmu = 0.505
    gamma = 3.0        # AR pre-emption gain.
    amax = 5.0         # Robot longitudinal acceleration limit [m/s^2].
    bmax = 0.28        # Robot slip-angle limit [rad].
    vmin = 1.0         # Minimum simulated robot speed [m/s].
    vmax = 3.5         # Maximum simulated robot speed [m/s].
    vdes = 2.5         # Desired cruise speed [m/s].
    alpha0 = 2.0       # Class-K gain: alpha(h) = alpha0 * h.
    r = 1.0            # Combined robot/obstacle safety radius [m].
    sense = 15.0       # Obstacles beyond this range are ignored by the QP [m].

    # soft-penalty params
    rho = 10.0
    eps_b = 0.3        # Width of the Huber-style buffer [barrier units].

    # reference go-to-goal gains
    Kv = 1.5
    Kth = 2.0


# ---------------------------------------------------------------------------
# Dynamics
# ---------------------------------------------------------------------------
def robot_deriv(xr, u):
    """Kinematic-bicycle dynamics."""
    x, y, th, v = xr
    a, be = u
    c, s = np.cos(th), np.sin(th)

    # The beta-dependent terms give the position channel the lateral slip
    # component used by the kinematic-bicycle model.
    return np.array([
        v * c - v * be * s,
        v * s + v * be * c,
        (v / P.lr) * be,
        a,
    ])


def robot_step(xr, u, dt):
    """RK4 step for the robot dynamics."""
    k1 = robot_deriv(xr, u)
    k2 = robot_deriv(xr + 0.5 * dt * k1, u)
    k3 = robot_deriv(xr + 0.5 * dt * k2, u)
    k4 = robot_deriv(xr + dt * k3, u)

    xr = xr + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
    xr[3] = np.clip(xr[3], P.vmin, P.vmax)
    return xr


def obs_deriv(xo, aw):
    """Unicycle obstacle dynamics."""
    x, y, th, v = xo
    a, w = aw
    return np.array([
        v * np.cos(th),
        v * np.sin(th),
        w,
        a,
    ])


def obs_step(xo, aw, dt):
    """RK4 step for the obstacle dynamics."""
    k1 = obs_deriv(xo, aw)
    k2 = obs_deriv(xo + 0.5 * dt * k1, aw)
    k3 = obs_deriv(xo + 0.5 * dt * k2, aw)
    k4 = obs_deriv(xo + dt * k3, aw)

    xo = xo + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
    xo[3] = max(xo[3], 0.0)
    return xo


# ----------------------------- barrier ---------------------------------
def barrier(xr, xo, kappa, adversarial):
    """Return h (or h*) for the current robot-obstacle pair."""
    # Relative position and LoS frame.
    p_rel = xo[:2] - xr[:2]
    nP = max(np.hypot(p_rel[0], p_rel[1]), 1e-6)
    xhat = p_rel / nP
    yhat = np.array([-xhat[1], xhat[0]])

    # DPCBF uses the no-slip robot velocity.
    vrob = xr[3] * np.array([np.cos(xr[2]), np.sin(xr[2])])
    vobs = xo[3] * np.array([np.cos(xo[2]), np.sin(xo[2])])
    vrel = vobs - vrob

    # Relative velocity components in the line-of-sight frame.
    vtx = vrel @ xhat
    vty = vrel @ yhat

    # Tangency distance associated with the combined safety radius.
    d = np.sqrt(max(nP * nP - P.r * P.r, 1e-9))

    # Engagement floor on ||v_rel||.
    nv = max(np.hypot(vrel[0], vrel[1]), 0.2)

    lam = P.klam * d / nv
    mu = P.kmu * d

    if adversarial and kappa > 0.0:
        # Subtract the manoeuvre buffer and clamp at zero.
        lam = max(lam - kappa / (P.gamma * P.amax * nv), 0.0)
        mu = max(mu - kappa * d / (P.gamma * nv), 0.0)

    return vtx + lam * vty * vty + mu


def _part(xr, xo, kappa, adv, which, eps=1e-6):
    """Central-difference partials of the barrier."""
    out = {}
    for fr, i in which:
        if fr == 'r':
            dr = np.zeros(4)
            dr[i] = eps
            out[(fr, i)] = (
                barrier(xr + dr, xo, kappa, adv)
                - barrier(xr - dr, xo, kappa, adv)
            ) / (2 * eps)
        else:
            do = np.zeros(4)
            do[i] = eps
            out[(fr, i)] = (
                barrier(xr, xo + do, kappa, adv)
                - barrier(xr, xo - do, kappa, adv)
            ) / (2 * eps)
    return out


def lie(xr, xo, kappa, adversarial):
    """(Lf, Lg[2], h) for the CBF-QP."""
    _, _, th, v = xr
    c, s = np.cos(th), np.sin(th)

    # Only the needed state partials.
    g = _part(
        xr, xo, kappa, adversarial,
        [('r', 0), ('r', 1), ('r', 2), ('r', 3), ('o', 0), ('o', 1)],
    )

    # Translational drift terms.
    Lf = (
        g[('r', 0)] * (v * c)
        + g[('r', 1)] * (v * s)
        + g[('o', 0)] * (xo[3] * np.cos(xo[2]))
        + g[('o', 1)] * (xo[3] * np.sin(xo[2]))
    )

    # Control channels corresponding to acceleration and beta.
    Lg_a = g[('r', 3)]
    Lg_beta = (
        g[('r', 0)] * (-v * s)
        + g[('r', 1)] * (v * c)
        + g[('r', 2)] * (v / P.lr)
    )

    h = barrier(xr, xo, kappa, adversarial)
    return Lf, np.array([Lg_a, Lg_beta]), h, None, None


def lie_Lg(xr, xo, kappa, adversarial):
    """Return only the control Lie derivatives and barrier value.

    This reduced derivative calculation is used by the soft and buffer
    penalty terms, where the drift term is not required.
    """
    _, _, th, v = xr
    c, s = np.cos(th), np.sin(th)

    g = _part(
        xr, xo, kappa, adversarial,
        [('r', 0), ('r', 1), ('r', 2), ('r', 3)],
    )

    Lg_a = g[('r', 3)]
    Lg_beta = (
        g[('r', 0)] * (-v * s)
        + g[('r', 1)] * (v * c)
        + g[('r', 2)] * (v / P.lr)
    )

    return np.array([Lg_a, Lg_beta]), barrier(xr, xo, kappa, adversarial)


def grad_barrier(xr, xo, kappa, adversarial, eps=1e-6):
    """Return numerical gradients with respect to the full robot/obstacle states.

    This helper is primarily intended for derivative sanity checks rather
    than the main control loop.
    """
    g = _part(
        xr, xo, kappa, adversarial,
        [(f, i) for f in 'ro' for i in range(4)],
        eps,
    )
    return (
        np.array([g[('r', i)] for i in range(4)]),
        np.array([g[('o', i)] for i in range(4)]),
    )


# ---------------------------------------------------------------------------
# Two-dimensional QP projection
# ---------------------------------------------------------------------------
def qp_project(uref, A, b, lo, hi):
    """Project a reference control onto the feasible control set.

    Solves

        min ||u - uref||^2
        subject to A u >= b and lo <= u <= hi.

    Because the control has only two components, the projection can be found
    by enumerating projections onto individual half-planes and intersections
    of pairs of constraint boundaries.  This avoids introducing a separate
    numerical QP dependency.

    Returns
    -------
    u : ndarray, shape (2,)
        Projected control.
    feasible : bool
        ``True`` if a point satisfying every constraint was found.  If the
        intersection is infeasible, the function returns the box-clamped
        reference control and sets this flag to ``False``.
    """
    uref = np.asarray(uref, float)

    # Convert the input box into four half-plane constraints so every
    # feasibility test uses the same representation.
    Aall = [
        np.array([1.0, 0.0]),
        np.array([-1.0, 0.0]),
        np.array([0.0, 1.0]),
        np.array([0.0, -1.0]),
    ]
    ball = [lo[0], -hi[0], lo[1], -hi[1]]

    for i in range(len(b)):
        Aall.append(np.asarray(A[i], float))
        ball.append(float(b[i]))

    Aall = np.array(Aall)
    ball = np.array(ball)
    tol = 1e-7

    def feasible(u):
        """Check all half-plane constraints with a small numerical tolerance."""
        return np.all(Aall @ u >= ball - tol)

    # The unconstrained reference is already the minimum-distance solution.
    if feasible(uref):
        return uref.copy(), True

    candidates = []

    # Project the reference onto each individual constraint boundary.
    for i in range(len(ball)):
        ai = Aall[i]
        nrm2 = ai @ ai
        if nrm2 < 1e-12:
            continue
        u = uref + (ball[i] - ai @ uref) / nrm2 * ai
        candidates.append(u)

    # In two dimensions, an optimum can also occur at the intersection of
    # two active constraint boundaries.
    n = len(ball)
    for i in range(n):
        for j in range(i + 1, n):
            M = np.array([Aall[i], Aall[j]])
            det = np.linalg.det(M)
            if abs(det) < 1e-9:
                continue
            u = np.linalg.solve(M, np.array([ball[i], ball[j]]))
            candidates.append(u)

    best = None
    bestd = np.inf
    for u in candidates:
        if feasible(u):
            dd = np.sum((u - uref) ** 2)
            if dd < bestd:
                bestd = dd
                best = u

    if best is None:
        # No feasible point exists.  Keep the simulation running with the
        # closest point in the actuator box and explicitly report infeasibility.
        u = np.array([
            np.clip(uref[0], lo[0], hi[0]),
            np.clip(uref[1], lo[1], hi[1]),
        ])
        return u, False

    return best, True


# ---------------------------------------------------------------------------
# Nominal and AR-DPCBF controllers
# ---------------------------------------------------------------------------
def reference_control(xr, goal):
    """Compute the nominal go-to-goal reference control ``[a, beta]``."""
    dx = goal[0] - xr[0]
    dy = goal[1] - xr[1]
    th_goal = np.arctan2(dy, dx)

    # Wrap heading error to [-pi, pi] before applying the proportional gain.
    err = np.arctan2(
        np.sin(th_goal - xr[2]),
        np.cos(th_goal - xr[2]),
    )

    a = P.Kv * (P.vdes - xr[3])
    be = P.Kth * err

    return np.array([
        np.clip(a, -P.amax, P.amax),
        np.clip(be, -P.bmax, P.bmax),
    ])


def dphi_soft(s):
    """Derivative of the quadratic soft penalty ``max(0, -s)^2``."""
    return 2.0 * min(0.0, s)


def dphi_buffer(s, eps_b):
    """Derivative of the Huber-style AR-DPCBF buffer penalty.

    The derivative is zero outside the positive buffer, decreases linearly
    inside the buffer, and saturates at -1 for a violated barrier.
    """
    if s > eps_b:
        return 0.0
    if s > 0.0:
        return -(eps_b - s) / eps_b
    return -1.0


def compute_control(method, xr, obs_list, goal, kappa):
    """Compute one control action for a selected AR-DPCBF controller.

    Parameters
    ----------
    method : {'dpcbf', 'hard', 'soft', 'buffer'}
        Controller variant:
          * ``dpcbf``: nominal DPCBF is enforced as a hard constraint.
          * ``hard``: contracted AR-DPCBF is enforced as a hard constraint.
          * ``soft``: nominal DPCBF is hard; AR-DPCBF is a quadratic penalty.
          * ``buffer``: nominal DPCBF is hard; AR-DPCBF uses the Huber buffer.
    xr : array-like
        Current robot state.
    obs_list : sequence
        Current obstacle states.
    goal : array-like, shape (2,)
        Position of the goal.
    kappa : float
        Obstacle maneuver capability used for AR-DPCBF contraction.

    Returns
    -------
    u : ndarray, shape (2,)
        Control ``[a, beta]``.
    feasible : bool
        Feasibility flag returned by the underlying projection.  For ``hard``,
        ``False`` specifically indicates that the AR-DPCBF QP was infeasible
        and the controller fell back to the nominal DPCBF QP.
    """
    uref = reference_control(xr, goal)
    lo = np.array([-P.amax, -P.bmax])
    hi = np.array([P.amax, P.bmax])

    # Only sensed obstacles contribute constraints or penalties.
    near = [
        xo for xo in obs_list
        if np.hypot(xo[0] - xr[0], xo[1] - xr[1]) <= P.sense
    ]

    def dpcbf_constraints():
        """Build nominal DPCBF inequalities in the form A u >= b."""
        A = []
        b = []
        for xo in near:
            Lf, Lg, h, _, _ = lie(
                xr, xo, kappa, adversarial=False
            )
            # Lf + Lg u + alpha(h) >= 0.
            A.append(Lg)
            b.append(-P.alpha0 * h - Lf)
        return A, b

    if method == 'dpcbf':
        A, b = dpcbf_constraints()
        return qp_project(uref, A, b, lo, hi)

    if method == 'hard':
        A = []
        b = []
        for xo in near:
            Lf, Lg, h, _, _ = lie(
                xr, xo, kappa, adversarial=True
            )
            # The contracted barrier replaces h by h* in the hard CBF constraint.
            A.append(Lg)
            b.append(-P.alpha0 * h - Lf)

        u, feas = qp_project(uref, A, b, lo, hi)

        if not feas:
            # Preserve the nominal DPCBF fallback used by the experiments.
            Ad, bd = dpcbf_constraints()
            u, _ = qp_project(uref, Ad, bd, lo, hi)
            return u, False

        return u, True

    # For soft and buffer variants, the nominal DPCBF remains the hard
    # constraint.  The contracted AR-DPCBF contributes only to the objective.
    A, b = dpcbf_constraints()
    qpen = np.zeros(2)

    for xo in near:
        Lgs, hs = lie_Lg(
            xr, xo, kappa, adversarial=True
        )
        d = (
            dphi_soft(hs)
            if method == 'soft'
            else dphi_buffer(hs, P.eps_b)
        )
        qpen += P.rho * d * Lgs

    # Completing the square in
    # ||u-uref||^2 + qpen^T u gives the shifted reference below.
    uref_eff = uref - 0.5 * qpen
    return qp_project(uref_eff, A, b, lo, hi)


# ---------------------------------------------------------------------------
# Worst-case obstacle maneuver
# ---------------------------------------------------------------------------
def adversary_input(xr, xo, kappa, aobs_max, wobs_max):
    """Return the worst-case obstacle maneuver for the current barrier.

    The obstacle chooses acceleration and angular rate from the admissible
    box to minimize the instantaneous DPCBF barrier rate.  Since the barrier
    rate is affine in these two obstacle inputs, the minimizer is attained at
    a box corner.  The signs are determined from the barrier sensitivities
    with respect to obstacle speed and heading.
    """
    g = _part(
        xr, xo, kappa, False,
        [('o', 2), ('o', 3)],
    )
    dh_dtho = g[('o', 2)]
    dh_dvo = g[('o', 3)]

    # Move in the direction that decreases h as rapidly as possible.
    a = (
        -aobs_max * np.sign(dh_dvo)
        if abs(dh_dvo) > 1e-12
        else 0.0
    )
    w = (
        -wobs_max * np.sign(dh_dtho)
        if abs(dh_dtho) > 1e-12
        else 0.0
    )
    return np.array([a, w])
