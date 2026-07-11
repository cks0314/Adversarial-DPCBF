<div align='center'>
<h2 align="center"> AR-DPCBF: Adversarial-Robust Dynamic Parabolic Control Barrier Functions </h2>

**Safe navigation for nonholonomic robots against *maneuvering* obstacles.**

Reference implementation for the paper *"Adversarial-Robust Dynamic Parabolic Control Barrier
Functions for Nonholonomic Robots Against Maneuvering Obstacles."*

<p align="center">
  <img src="results/readme_media/hero_buffer.gif" width="100%" alt="Four controllers on one scenario: DPCBF collides, AR-DPCBF variants reach the goal"/>
</p>

<p align="center">
  <em>Same scenario, four controllers. DPCBF's QP reports its barrier satisfied the whole time — and the
  robot still hits an obstacle. The three AR-DPCBF variants route around the same threat and reach the goal.</em>
</p>
<div align='center'>
---
</div>
## Motivation

Control Barrier Functions certify safety by enforcing `ḣ ≥ −α(h)` in a QP. The Dynamic Parabolic CBF
(DPCBF) does this elegantly for dynamic obstacles: it replaces the conservative collision cone with a
state-dependent parabola in the line-of-sight (LoS) velocity frame, recovering feasibility in clutter
where cone-based methods have none.

But DPCBF — like every existing CBF for dynamic obstacle avoidance — evaluates its Lie derivative
assuming the obstacle **holds constant velocity**. If the obstacle accelerates or turns, the true
derivative picks up an uncompensated term:

```
[ḣ]_true = [ḣ]_DPCBF + Δ(t)
Δ(t) = a_obs·cos θ̃_obs + v_obs·ω_obs·( 2λ ṽ_rel,y cos θ̃_obs − sin θ̃_obs )
```

When `Δ(t) < 0`, the CBF condition can be satisfied by the QP **while being violated in reality**.
There is no infeasibility, no warning — the safety guarantee simply **fails silently**, and the first
symptom is a collision. This repository formalises that failure mode and fixes it.

---

## Key contributions

1. **The silent-failure mode**, formalised — and the **Adversarial CBF (A-CBF)** as the correct
   validity notion when the obstacle is a bounded adversary rather than a constant-velocity mover.
2. **A zero-sum differential game** in the LoS frame, with a reducibility lemma collapsing the full
   state space to a tractable two-dimensional sub-game.
3. **Closed-form adversarially-robust barrier parameters** `(λ*, μ*)` — no HJI PDE, no grid, no loss
   of real-time performance. Per-cycle cost is identical to DPCBF.
4. **A validity theorem** (`κ < c_min`) certifying the barrier is maintainable against *every*
   admissible obstacle maneuver — verified empirically with **zero counterexamples**.
5. **Two soft variants** that keep the DPCBF constraint hard and penalise the adversarial barrier in
   the objective, recovering the feasibility that hard enforcement destroys in dense scenes.
6. **An online capability estimator** with sub-Gaussian coverage guarantees, so `κ` need not be known
   in advance.

---

## Method overview

The obstacle is modelled as a bounded adversary with capability set
`F = { |a_obs| ≤ a_obs,max , |ω_obs| ≤ ω_obs,max }`, summarised by a single scalar:

```
κ := a_obs,max + v_obs,max · ω_obs,max        [m/s²]
```

AR-DPCBF **contracts** the DPCBF parabola by subtracting a worst-case maneuver budget from *both*
gains (Theorem 2):

```
DPCBF     h  = ṽ_rel,x + λ  ṽ_rel,y² + μ        λ  = k_λ d/‖v_rel‖              μ  = k_μ d
AR-DPCBF  h* = ṽ_rel,x + λ* ṽ_rel,y² + μ*       λ* = λ − κ/(γ a_max ‖v_rel‖)    μ* = μ − κ d/(γ ‖v_rel‖)
```

<p align="center">
  <img src="results/readme_media/variation.gif" width="94%" alt="Parabola contraction with kappa, relative speed and clearance"/>
</p>

<p align="center">
  <em><b>Left:</b> the true unsafe sets over robot positions — orange <code>{h&lt;0}</code> (DPCBF) always lies
  <em>inside</em> green <code>{h*&lt;0}</code> (AR-DPCBF). <b>Right:</b> the same objects in the LoS velocity frame.
  Since λ* ≤ λ and μ* ≤ μ, the AR boundary sits to the <b>right</b> of DPCBF's: the safe set shrinks as the
  obstacle grows more capable, and recovers DPCBF <b>exactly</b> at κ = 0.</em>
</p>

**Why subtract rather than add?** Contraction is what makes the certificate self-contained:
`{h* ≥ 0} ⊆ {h ≥ 0}`, so defending `h*` *inherits* DPCBF's clearance property (Proposition 3). Adding
the correction would enlarge the safe set and break the inclusion — robustness would then rest
entirely on the runtime margin.

---

## Theoretical highlights

| result | statement |
|---|---|
| **Def. 4 — A-CBF** | `h*` is an *adversarial* CBF if for all `x ∈ C*` and **all** `(a_obs, ω_obs) ∈ F` there exists `u ∈ U` with `L_f h*(x, a_obs, ω_obs) + L_g h*(x)·u ≥ −α(h*)`. Worst-case over the obstacle, best-case over the robot. |
| **Thm. 2 — contraction** | `λ* ≤ λ`, `μ* ≤ μ`, both monotone in `κ`; **exact recovery** `h* ≡ h` at `κ = 0`; and a **validity floor** past which the parabola would invert (`λ*, μ*` clamp at 0). |
| **Prop. 3 — safety inheritance** | `{h* ≥ 0} ⊆ {h ≥ 0}`, hence maintaining `h* ≥ 0` implies clearance `d(t) ≥ r` for all `t`. |
| **Thm. 9 — validity** | If the robot **out-authorises** the obstacle — `κ < c_min` and `(c_min − κ)·Λ_min ≥ ε − D_min` — then `C* = {h* ≥ 0}` is forward invariant under *every* obstacle in `F`, where `c_min = min{ a_max , v_min² β_max / ℓ_r }`. |
| **Props. 11–12 — feasibility** | The soft variants' feasible set **equals** DPCBF's: they keep the DPCBF constraint hard and penalise `h*` in the objective only. Hard AR-DPCBF does not, and loses feasibility in clutter. |
| **Prop. 10 — estimator** | With sub-Gaussian sensor noise and margins `(ε_a, ε_ω, ε_v)`, the online estimate satisfies `P[κ̃ ≥ κ] ≥ 1 − δ`. By monotonicity in `κ`, over-coverage yields a *more conservative, still safe* controller. |

### The four controllers

| controller | hard constraint | objective penalty | feasibility |
|---|---|---|---|
| **DPCBF** (baseline) | `h ≥ 0` | — | full |
| **Hard AR** | `h* ≥ 0` | — | **reduced** when `κ > 0` |
| **Soft AR** | `h ≥ 0` | `ρ · max(0, −h*)²` | full |
| **Buffer Soft AR** | `h ≥ 0` | `ρ · φ(h*; ε)` — Huber buffer | full |

The Huber buffer `φ` is the decisive refinement. The plain soft penalty has **zero gradient** while
`h* > 0`: the solver gets no steering signal until the adversarial boundary has *already* been
breached. The buffer supplies a non-zero gradient in the band `0 < h* ≤ ε`, steering the robot away
**before** the boundary is crossed.

---

## Results

### 1. The silent failure is real

<p align="center"><img src="assets/fig_silent_case.png" width="100%"/></p>

One deterministic scenario. DPCBF's barrier `h` stays **≥ 0 for the entire run** — its QP never
reports a problem — yet the true clearance `d − r` goes negative (see inset). Buffer Soft AR-DPCBF,
on the identical scenario, routes around the same adversary and reaches the goal.

### 2. It scales with the threat — and AR removes it

<p align="center"><img src="assets/fig_sweep15_ci_fix.png" width="100%"/></p>

Adversary-count sweep at `N = 15` (95% bootstrap CIs). DPCBF's **barrier-violation rate** climbs to
**95%** and its collision rate to **60%**, while all AR variants stay low. The wide gap between the
violation and collision curves is itself the signature of *silent* failure: the certificate is
breached far more often than contact actually occurs.

Pooled over six dense conditions (120 paired trials, exact McNemar test):

| metric | DPCBF | Hard AR | Soft AR | Buffer Soft AR |
|---|---|---|---|---|
| barrier violation | 84% | 46% | 29% | **25%** |
| collision | 49% | 35% | 24% | **21%** |

Buffer Soft AR-DPCBF is the only variant that significantly improves on **both** DPCBF and Hard AR on
**both** axes (`p < 10⁻³`).

### 3. Ablation on three deterministic cases

Each row is a single seed; all four controllers face the *identical* obstacle field
(clearance at closest approach in parentheses — negative means collision):

| seed | DPCBF | Hard AR | Soft AR | Buffer Soft AR | what it isolates |
|---|---|---|---|---|---|
| **13** | ✗ (−0.92 m) | ✓ (+0.61) | ✓ (+0.52) | ✓ (+0.93) | the adversarial barrier alone fixes DPCBF |
| **17** | ✗ (−0.75) | ✗ (−0.71) | ✓ (+0.64) | ✓ (+0.91) | soft penalties add what hard enforcement cannot |
| **7** | ✗ (−0.71) | ✗ (−0.85) | ✗ (−0.39) | ✓ (**+1.02**) | the proactive buffer is decisive in the hardest case |

<p align="center">
  <img src="assets/demo_compare_7.gif" width="100%" alt="Seed 7: only Buffer Soft AR-DPCBF survives"/>
</p>
<p align="center"><em>Seed 7 — DPCBF, Hard AR and Soft AR all collide; only Buffer Soft AR-DPCBF gets through.</em></p>

### 4. Feasibility — why the soft variants exist

<p align="center"><img src="assets/fig_density.png" width="100%"/></p>

Hard AR-DPCBF's QP-infeasibility rate rises with density (→ ~4.9% at `N = 30`) while Soft (~0.8%) and
Buffer (~0.3%) stay flat near zero — the empirical signature of Propositions 11–12. Note the soft
variants sit *below* DPCBF: Props 11–12 guarantee equality of the feasible set **at a given state**,
and because the penalties steer proactively, the robot simply *visits* near-infeasible pockets less
often.

### 5. The main theorem holds

<p align="center"><img src="assets/fig_boundary.png" width="74%"/></p>

The Nagumo condition `sup_u inf_F ḣ* ≥ 0` tested at sampled boundary states `{h* = 0}` across the
`(κ, c_min)` plane. **The entire region certified by Theorem 9 (`c_min > κ`, above the dashed line) is
100% maintainable — zero counterexamples.** The empirical cliff lies *beyond* the line and the
transition is graceful: the theorem is sound and mildly conservative, exactly as a sufficient
condition should be.

### 6. It fails safe

<p align="center"><img src="assets/fig_misspec.png" width="100%"/></p>

The controller assumes `κ = 0.98` while the obstacle's *true* capability is swept. Buffer Soft
AR-DPCBF holds violations near 13% even when the obstacle is **twice as capable as assumed** — a
graceful degradation with no cliff. Over-provisioning (`κ_true < κ_assumed`) is conservative but
*safer* than a matched controller.

### 7. The price of robustness

| | path length | time-to-goal | median QP cost | collisions |
|---|---|---|---|---|
| DPCBF | 50.1 m | 20.6 s | **78** (highest) | 42% |
| Buffer Soft AR | 53.6 m (+7%) | 21.8 s (+6%) | 47 | **8%** |

Robustness costs a ~7% path detour — but *reduces* median control effort. DPCBF is the **most**
effortful of the four, because under a maneuvering adversary it reacts late and violently, and the
squared cost punishes those spikes.

### 8. A negative result, kept on the record

A symmetric **all-sides encirclement** does *not* separate DPCBF from the soft variants: below a
capability threshold every controller escapes; above it every controller collides — and the more
conservative variants can be *trapped* and do **worse**. AR-DPCBF's advantage is keeping margin *when
there is room to route around a threat*; a closing ring removes that room, and tends toward an
inevitable-collision state. See [`docs/notes_encirclement.md`](docs/notes_encirclement.md) and
`experiments/encirclement_probe.py`.

---

## Installation

```bash
git clone https://github.com/<user>/ar-dpcbf.git
cd ar-dpcbf
pip install -r requirements.txt        # numpy, matplotlib
# ffmpeg is needed only for the .mp4 animation scripts
```

There is **no solver dependency**: the 2-D QP is solved exactly by KKT / vertex enumeration in
`ardpcbf_core.py`.

---

## Reproducing the paper

Figures that re-simulate on the fly (no data step needed):

```bash
python figures/plot_silent_case.py             # the silent-failure contrast
python figures/plot_variation.py               # parabola contraction
python figures/plot_t0.py                      # scenario initial condition
python figures/plot_timelapse.py buffer 57     # navigation timelapse (any method/seed)
python figures/animate_compare.py 13           # 2x2 four-controller MP4
python figures/animate_variation.py            # 30 s parameter-sweep MP4
```

Experiments (write `data/*.npz`), then their plotters:

```bash
python experiments/run_adversary_sweep.py 40  && python figures/plot_adversary_sweep.py
python experiments/run_core_sweeps.py all 40  && python figures/plot_core_sweeps.py all
python experiments/validity_boundary.py       && python figures/plot_boundary.py

bash run_all.sh                                # all of the above
```

### Determinism

Every scenario is a pure function of an integer seed:

```python
from scenario import make_scn
scn = make_scn(seed=13, N=10, n_adv=5)   # the n_adv obstacles CLOSEST to the robot's
                                         # start are designated the adversaries
```

### Parameters

`class P` in `ardpcbf/ardpcbf_core.py` holds the DPCBF Table I values — `ℓ_r = 0.20`,
`a_max = 5.0`, `β_max = 0.28`, `v_max = 3.5`, `v_des = 2.5`, `r = 1.0`, sensing radius `15` —
plus the three AR-specific settings, disclosed explicitly:

- `v_min = 1.0` — steering authority scales as `v²/ℓ_r` and vanishes at rest, so a positive floor is
  required to defend the barrier. This gives `c_min = min{a_max, v_min² β_max/ℓ_r} = 1.40`.
- `γ = 3.0` — the pre-emption gain (fraction of the worst-case maneuver reserved in the parameters).
- `ρ = 10.0`, `ε = 0.3` — soft-penalty weight and Huber buffer width.

### Metrics

Three axes, deliberately **not** conflated:

- **barrier violation** — `min_t h < 0`, the silent-failure event (**primary metric**)
- **collision** — `d < r`, its downstream consequence
- **QP infeasibility** — per-cycle rate at which the safety program admits no input

### Repository layout

```
ardpcbf/                 core library
  ardpcbf_core.py          dynamics, barriers, Lie derivatives, 2-D QP, four controllers, adversary
  ardpcbf_run.py           episode runner
  scenario.py              canonical scenario generator (nearest-adversary)
  ardpcbf_estimator.py     online capability estimator (Prop. 10)
experiments/             data runners        -> data/*.npz
figures/                 plotters, animators -> results/*.{pdf,png,mp4}
paper/                   Results section (LaTeX) + standalone preview
results/                 reference figures and videos
assets/                  GIFs and images used by this README
docs/                    figure index, notes
```

All figure PDFs embed editable TrueType fonts (`pdf.fonttype = 42`) and open as **editable vector
art** in Illustrator. See [`docs/figure_index.md`](docs/figure_index.md) for the exact script → figure
mapping.

---

## Citation

```bibtex
@article{ardpcbf2026,
  title   = {Adversarial-Robust Dynamic Parabolic Control Barrier Functions for
             Nonholonomic Robots Against Maneuvering Obstacles},
  author  = {<authors>},
  journal = {<venue>},
  year    = {2026}
}
```

Built on the DPCBF framework:

```bibtex
@inproceedings{park2026dpcbf,
  title     = {Beyond Collision Cones: Dynamic Obstacle Avoidance for Nonholonomic
               Robots via Dynamic Parabolic Control Barrier Functions},
  author    = {Park, H. K. and Kim, T. and Panagou, D.},
  booktitle = {IEEE Int. Conf. on Robotics and Automation (ICRA)},
  year      = {2026}
}
```

## License

MIT — see [LICENSE](LICENSE).
