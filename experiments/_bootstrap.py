"""Shared path bootstrap: import from the ardpcbf library and locate data/results dirs.
Import this first in any figure or experiment script:  ``from _bootstrap import ROOT, DATA, FIGS``.
"""
import sys, pathlib
ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ardpcbf"))
DATA = ROOT / "data"; DATA.mkdir(exist_ok=True)
FIGS = ROOT / "results"; FIGS.mkdir(exist_ok=True)
import matplotlib
matplotlib.use("Agg")
matplotlib.rcParams["pdf.fonttype"] = 42   # editable text in Illustrator
matplotlib.rcParams["ps.fonttype"] = 42
