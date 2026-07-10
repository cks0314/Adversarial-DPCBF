# Note: symmetric encirclement is a negative result

We investigated a DCBF-Fig.6-style scenario: the robot starts at the centre of
a ring of adversarial obstacles and must break out to a goal outside the ring
(`experiments/encirclement_probe.py`). It does **not** yield a clean
DPCBF-fails / soft-AR-succeeds contrast, for a structural reason:

- Below a capability threshold, **all four controllers escape**.
- Above it, **all four collide** — and because a tight ring rewards committing
  to a gap rather than hesitating, the more conservative variants (Hard/Soft/
  Buffer) can be *trapped* and do *worse* than DPCBF.
- Aggressive inward-homing rings become inevitable-collision states (ICS),
  which no controller can escape, confounding the comparison.

AR-DPCBF's advantage is keeping margin from a maneuvering obstacle **when there
is room to route around it**; a full encirclement removes that room. The
deterministic contrast is therefore shown in a *traversal*
(`figures/plot_silent_case.py`, seed 9): DPCBF satisfies its QP barrier
(`h >= 0`) yet breaches the safety radius, while Buffer Soft AR-DPCBF routes
around the same threat and reaches the goal.
