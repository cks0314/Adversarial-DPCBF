"""Validation of the online capability estimator (Section VI, Prop. 10)."""
import numpy as np
import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt

import ardpcbf_core as C

P = C.P

plt.rcParams.update({
    'font.size': 9,
    'axes.grid': True,
    'grid.alpha': 0.3,
    'figure.dpi': 130,
    'savefig.dpi': 200,
    'lines.linewidth': 1.8,
})


def estimator(vmeas, thmeas, dt_est, Nw, sv, sth, delta):
    """Estimate obstacle maneuver capability from noisy speed/heading data.

    A trailing window is used to estimate acceleration and angular-rate
    magnitudes from finite differences.  Noise margins are then added to the
    observed acceleration, angular rate, and speed before forming

        kappa_tilde = a_hat + v_hat * omega_hat.

    Parameters
    ----------
    vmeas, thmeas : array-like
        Measured obstacle speed and heading.
    dt_est : float
        Sampling period of the estimator, which may be coarser than the
        controller sampling period.
    Nw : int
        Number of estimator samples in the trailing window.
    sv, sth : float
        Standard-deviation parameters for speed and heading measurement noise.
    delta : float
        Target failure probability used in the concentration margin.

    Returns
    -------
    ndarray
        Capability estimate at every estimator sample.  The first sample may
        be ``NaN`` because a finite difference is not yet available.
    """
    # Common concentration factor used by all three noise margins.
    L = np.sqrt(2 * np.log(3 * Nw / delta))

    # Convert measurement uncertainty into conservative margins for
    # acceleration, angular rate, and speed.
    ea = (sv * np.sqrt(2) / dt_est) * L
    ew = (sth * np.sqrt(2) / dt_est) * L
    ev = sv * L

    n = len(vmeas)
    kap = np.full(n, np.nan)

    for k in range(n):
        # Window includes the current sample and at most Nw samples.
        lo = max(1, k - Nw + 1)

        dv = np.abs(np.diff(vmeas[lo - 1:k + 1])) / dt_est
        dth = (
            np.abs(np.diff(np.unwrap(thmeas[lo - 1:k + 1])))
            / dt_est
        )

        ahat = dv.max() if len(dv) else 0.0
        what = dth.max() if len(dth) else 0.0
        vhat = np.abs(vmeas[lo - 1:k + 1]).max()

        kap[k] = (ahat + ea) + (vhat + ev) * (what + ew)

    return kap


def _adv_trace(seed, aob, wob, vnom, dt, T, vcap=2.5):
    """Simulate a capability-saturating adversary and record its state.

    The obstacle applies the worst-case input from ``C.adversary_input``.
    The returned arrays contain the obstacle speed and heading at the
    controller sampling rate; the caller performs estimator-rate subsampling.
    """
    rng = np.random.default_rng(seed)
    n = int(T / dt)

    xr = np.array([0, 0, 0.0, P.vdes])
    xo = np.array([
        10.0,
        rng.uniform(-2, 2),
        np.pi,
        vnom,
    ])

    # Capability used by the oracle adversary.
    kapc = aob + vcap * wob

    vt = []
    tt = []

    for _ in range(n):
        aw = C.adversary_input(xr, xo, kapc, aob, wob)
        xo = C.obs_step(xo, aw, dt)
        xo[3] = min(xo[3], vcap)

        # Move the robot toward a fixed goal so the adversary interacts with
        # a representative moving robot rather than a stationary target.
        xr = C.robot_step(
            xr,
            C.reference_control(xr, np.array([24, 0])),
            dt,
        )

        vt.append(xo[3])
        tt.append(xo[2])

    return np.array(vt), np.array(tt)


def run_coverage(
    seeds,
    sv,
    sth,
    Nw=15,
    delta=0.1,
    aob=0.5,
    wob=0.6,
    vnom=1.0,
    dt=0.05,
    M=4,
    vcap=2.5,
):
    """Run the estimator over multiple random trials.

    Returns the mean empirical domination rate, its standard error across
    trials, and the mean median ratio of estimated to noiseless capability.
    """
    dt_est = M * dt
    T = 12.0

    cov = []
    ratio = []
    rng = np.random.default_rng(100)

    for s in seeds:
        vt, tt = _adv_trace(
            s, aob, wob, vnom, dt, T, vcap
        )

        # Subsample to the estimator rate.
        vs = vt[::M]
        ts = tt[::M]

        vm = vs + rng.normal(0, sv, len(vs))
        tm = ts + rng.normal(0, sth, len(ts))

        # Noisy estimate with confidence margins.
        kap_hat = estimator(
            vm, tm, dt_est, Nw, sv, sth, delta
        )

        # The noiseless estimate provides the realized reference bound used
        # for the empirical domination check.
        kap_clean = estimator(
            vs, ts, dt_est, Nw, 0.0, 0.0, delta
        )

        valid = ~np.isnan(kap_hat)
        cov.append(np.mean(kap_hat[valid] >= kap_clean[valid]))
        ratio.append(
            np.nanmedian(
                kap_hat[valid] / np.maximum(kap_clean[valid], 1e-6)
            )
        )

    return (
        np.mean(cov),
        np.std(cov) / np.sqrt(len(seeds)),
        np.mean(ratio),
    )


def fig_estimator():
    """Generate and save the estimator coverage validation figure."""
    seeds = list(range(20))
    noises = [0.01, 0.02, 0.05, 0.10]
    delta = 0.1
    M = 4
    dt = 0.05

    covs = []
    errs = []
    ratios = []

    for sv in noises:
        # The experiment assumes heading measurements are three times less
        # noisy than speed measurements.
        m, e, rr = run_coverage(
            seeds,
            sv,
            sv / 3.0,
            Nw=15,
            delta=delta,
            M=M,
        )
        covs.append(m)
        errs.append(e)
        ratios.append(rr)

    fig, ax = plt.subplots(1, 2, figsize=(7.8, 3.2))

    # Panel (a): empirical coverage against the target 1-delta level.
    a = ax[0]
    a.errorbar(
        noises,
        covs,
        yerr=errs,
        marker='o',
        ms=5,
        capsize=3,
        color='#1f77b4',
        label='empirical coverage',
    )
    a.axhline(
        1 - delta,
        color='#d62728',
        ls='--',
        label=r'predicted $1-\delta=%.2f$' % (1 - delta),
    )
    a.set_xlabel(
        r'sensor noise $\sigma_v$ (m/s), '
        r'$\sigma_\theta=\sigma_v/3$'
    )
    a.set_ylabel(r'$P[\tilde\kappa \geq \kappa_{\mathrm{cap}}]$')
    a.set_title('(a) Estimator coverage (Prop. 10)')
    a.set_ylim(0.5, 1.04)
    a.legend(fontsize=7, loc='lower left')

    for x, r in zip(noises, ratios):
        a.annotate(
            r'$\tilde\kappa/\kappa{=}%.1f$' % r,
            (x, 1.005),
            fontsize=6,
            ha='center',
        )

    # Panel (b): compare one noisy estimate against the corresponding
    # noiseless realized maximum at moderate measurement noise.
    sv = 0.05
    vt, tt = _adv_trace(
        3, 0.5, 0.6, 1.0, dt, 12.0, 2.5
    )
    vs = vt[::M]
    ts_ = tt[::M]

    rng = np.random.default_rng(5)
    vm = vs + rng.normal(0, sv, len(vs))
    tm = ts_ + rng.normal(0, sv / 3, len(ts_))

    kh = estimator(
        vm, tm, M * dt, 15, sv, sv / 3, delta
    )
    kc = estimator(
        vs, ts_, M * dt, 15, 0.0, 0.0, delta
    )
    tax = np.arange(len(kh)) * M * dt

    b = ax[1]
    b.plot(
        tax,
        kh,
        color='#1f77b4',
        marker='.',
        ms=3,
        label=r'noisy estimate $\tilde\kappa(t)$',
    )
    b.plot(
        tax,
        kc,
        color='k',
        ls='--',
        label=r'noiseless realized $\hat\kappa(t)$',
    )
    b.fill_between(
        tax,
        kc,
        kh,
        where=kh >= kc,
        color='#2ca02c',
        alpha=0.2,
        label='dominates (safe)',
    )
    b.set_xlabel('t (s)')
    b.set_ylabel(r'$\kappa$ (m s$^{-2}$)')
    b.set_title(r'(b) Margins absorb noise ($\sigma_v{=}0.05$)')
    b.legend(fontsize=7)
    b.set_ylim(0, max(kh.max(), kc.max()) * 1.15)

    fig.tight_layout()
    fig.savefig('fig_estimator.pdf')
    fig.savefig('fig_estimator.png')
    plt.close(fig)

    print(
        "coverage:",
        [f"{c:.3f}" for c in covs],
        " ratios kappa~/kappa:",
        [f"{r:.1f}" for r in ratios],
    )


if __name__ == '__main__':
    fig_estimator()
