#!/usr/bin/env bash
# Regenerate the scripted figures. Fast figures re-simulate on the fly; the
# sweeps write data/*.npz first. Adjust seed counts (n) for speed vs. tightness.
set -e
echo "[1/4] self-contained figures"
python figures/plot_variation.py
python figures/plot_t0.py
python figures/plot_silent_case.py
echo "[2/4] validity boundary (Theorem 9)"
python experiments/validity_boundary.py && python figures/plot_boundary.py
echo "[3/4] adversary-count sweep (N=15)  [slow]"
python experiments/run_adversary_sweep.py 40 && python figures/plot_adversary_sweep.py
echo "[4/4] capability / density / gamma sweeps  [slow]"
python experiments/run_core_sweeps.py all 40 && python figures/plot_core_sweeps.py all
echo "done -> results/"
