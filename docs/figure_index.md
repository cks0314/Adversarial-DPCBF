# Figure index

Mapping from each paper figure to the code and data that produce it. All plot
scripts write to `results/` as both `.pdf` (editable vector) and `.png`.

## Fully scripted (runner + plotter bundled)

| figure | data runner | plotter |
|---|---|---|
| parabola contraction vs (κ,‖v_rel‖,d) | — (analytic) | `figures/plot_variation.py` |
| initial configuration t=0 | — (re-simulates) | `figures/plot_t0.py` |
| deterministic silent-failure contrast | — (re-simulates) | `figures/plot_silent_case.py` |
| adversary-count sweep (N=15) | `experiments/run_adversary_sweep.py` | `figures/plot_adversary_sweep.py` |
| capability sweep | `experiments/run_core_sweeps.py capability` | `figures/plot_core_sweeps.py capability` |
| density / feasibility (Props 11-12) | `experiments/run_core_sweeps.py density` | `figures/plot_core_sweeps.py density` |
| γ ablation | `experiments/run_core_sweeps.py gamma` | `figures/plot_core_sweeps.py gamma` |
| validity boundary (Theorem 9) | `experiments/validity_boundary.py` | `figures/plot_boundary.py` |

## Reference figures included in `results/` (plotters not bundled)

These final figures ship as reference outputs. Each is produced by the same
`run_episode` interface and metric conventions as the scripted experiments
above; the one-off plotting scripts are omitted here to avoid shipping
unverified reconstructions. They can be added on request.

| figure | how it is produced |
|---|---|
| paired McNemar (6 dense conditions, N=15) | per-trial collision/violation from the same episodes as the adversary sweep; exact two-sided McNemar test on common seeds |
| cost of robustness (path / time / QP cost) | per-episode `path`, `t_goal`, and QP objective at N=10, n_adv=5; QP cost reported as median with a log box plot (heavy-tailed) |
| κ-misspecification (fail-safe) | controller assumes κ=0.98 while the adversary's true κ is swept; decouple `kappa_ctrl` from the adversary's κ |
| closed-loop online estimator (Prop 10) | `ardpcbf/ardpcbf_estimator.py` feeds a per-cycle κ estimate into the controller; compared against an oracle-κ controller |
| Soft / Buffer navigation timelapses | single deterministic episodes with `record_trace=True`, rendered as time snapshots with the position-space unsafe sets |

## Metric conventions

- **Barrier violation** (primary): `min_t h(x(t)) < 0` — the silent-failure event.
- **Collision**: `d(t) < r`.
- **QP infeasibility**: per-cycle rate the safety program admits no input.
- **Success**: goal reached AND collision-free.
- Uncertainty: 95% bootstrap CIs (4000 resamples). Paired tests: exact McNemar.
