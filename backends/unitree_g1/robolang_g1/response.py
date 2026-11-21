"""What a correction does to a robot that stands still, as measured, and the choice of the correction from it.

`move`'s last centimetres are a servo (`ops.G1Operations._servo`): look at the station marker, command a short burst of
velocity, settle, look again. The walking policy takes steps, so a burst does not move the robot by what it is asked: a
sideways burst turns it too, a turn moves it sideways, a short burst may take no step. The tables below say what each
burst does, measured, and `plan_correction` picks the one that leaves the smallest error. Pure: no DDS, no ground truth
at run time (the numbers were measured with the oracle, offline).

A correction is one burst, or two sent one right after the other: ((vx, vy, wz, seconds), ...). Each table maps it to
the pose change of a robot that started standing still, SETTLE_LOOK_S after the correction ended (about when the look
that follows it is taken): (forward m, left m, heading rad) in the robot's own frame at the start, the mean of three
repeats, with the stance's own heading creep taken out (the plan adds it back, from what the robot sees).
`RESPONSE` is the robot with empty hands, `RESPONSE_CARRY` with the box in the right hand (the carry pose): they differ
by up to 0.16 m (a sidestep to the left that moves an empty robot 0.16 m moves a loaded one 0.00 m). Measured with
docs/evidence/g1-phase3/exp_move_response.py; raw trials in docs/evidence/g1-phase3/move_response_{empty,carry}.json;
means, spreads and the stance's creep in docs/evidence/g1-phase3-observations.md, "Shelf move convergence".
"""

from __future__ import annotations

SETTLE_LOOK_S = 3.0  # a correction's end to the look that follows it (the table's time), simulation seconds
CONFIRM_DWELL_S = 2.5  # the last servo look to the confirming look (a second settle and five frames)

Correction = tuple[tuple[float, float, float, float], ...]

# () is waiting: the heading creeps by itself, which can be all a small heading error needs.
RESPONSE: dict[Correction, tuple[float, float, float]] = {
    (): (0.0, 0.0, 0.0),
    ((-0.4, 0.0, -0.2, 0.2),): (-0.069, 0.023, -0.015),
    ((-0.4, 0.0, -0.2, 0.3),): (-0.111, 0.022, -0.026),
    ((-0.4, 0.0, -0.2, 0.4),): (-0.142, 0.03, -0.056),
    ((-0.4, 0.0, -0.2, 0.8),): (-0.248, 0.042, -0.073),
    ((-0.4, 0.0, 0.0, 0.2),): (-0.109, 0.046, -0.013),
    ((-0.4, 0.0, 0.0, 0.3),): (-0.103, 0.01, -0.02),
    ((-0.4, 0.0, 0.0, 0.4),): (-0.134, 0.014, -0.035),
    ((-0.4, 0.0, 0.0, 0.8),): (-0.246, 0.028, -0.009),
    ((-0.4, 0.0, 0.2, 0.2),): (-0.06, 0.006, -0.005),
    ((-0.4, 0.0, 0.2, 0.3),): (-0.122, -0.001, -0.01),
    ((-0.4, 0.0, 0.2, 0.4),): (-0.17, 0.004, -0.019),
    ((-0.4, 0.0, 0.2, 0.8),): (-0.272, 0.01, 0.051),
    ((0.0, -0.3, -0.2, 1.2),): (-0.052, -0.144, -0.121),
    ((0.0, -0.3, 0.2, 1.2),): (-0.023, -0.125, 0.096),
    ((0.0, 0.3, -0.2, 1.2),): (0.009, 0.159, -0.155),
    ((0.0, 0.3, 0.2, 1.2),): (-0.05, 0.127, 0.12),
    ((0.1, -0.3, -0.2, 0.5),): (0.026, -0.049, -0.04),
    ((0.1, -0.3, -0.2, 0.5), (0.1, 0.3, -0.2, 0.5)): (0.053, 0.047, -0.081),
    ((0.1, -0.3, -0.2, 0.8),): (0.059, -0.072, -0.063),
    ((0.1, -0.3, -0.2, 0.8), (0.1, 0.3, -0.2, 0.8)): (0.106, 0.0, -0.192),
    ((0.1, -0.3, -0.2, 1.2),): (0.063, -0.12, -0.121),
    ((0.1, -0.3, 0.0, 0.5),): (0.002, -0.005, -0.003),
    ((0.1, -0.3, 0.0, 0.8),): (0.062, -0.061, -0.019),
    ((0.1, -0.3, 0.0, 1.2),): (0.079, -0.148, -0.001),
    ((0.1, -0.3, 0.2, 0.5),): (0.003, -0.021, 0.018),
    ((0.1, -0.3, 0.2, 0.5), (0.1, 0.3, 0.2, 0.5)): (0.038, 0.015, 0.095),
    ((0.1, -0.3, 0.2, 0.8),): (0.062, -0.053, 0.071),
    ((0.1, -0.3, 0.2, 0.8), (0.1, 0.3, 0.2, 0.8)): (0.093, 0.071, 0.149),
    ((0.1, -0.3, 0.2, 1.2),): (0.077, -0.148, 0.124),
    ((0.1, 0.3, -0.2, 0.5),): (0.01, 0.027, -0.033),
    ((0.1, 0.3, -0.2, 0.5), (0.1, -0.3, -0.2, 0.5)): (0.014, 0.029, -0.06),
    ((0.1, 0.3, -0.2, 0.8),): (0.002, 0.004, -0.059),
    ((0.1, 0.3, -0.2, 0.8), (0.1, -0.3, -0.2, 0.8)): (0.015, -0.002, -0.089),
    ((0.1, 0.3, -0.2, 1.2),): (0.056, 0.06, -0.159),
    ((0.1, 0.3, 0.0, 0.5),): (0.006, 0.002, -0.018),
    ((0.1, 0.3, 0.0, 0.8),): (0.027, 0.045, -0.024),
    ((0.1, 0.3, 0.0, 1.2),): (0.058, 0.089, -0.024),
    ((0.1, 0.3, 0.2, 0.5),): (0.029, 0.055, 0.034),
    ((0.1, 0.3, 0.2, 0.5), (0.1, -0.3, 0.2, 0.5)): (-0.0, -0.003, 0.02),
    ((0.1, 0.3, 0.2, 0.8),): (0.037, 0.097, 0.074),
    ((0.1, 0.3, 0.2, 0.8), (0.1, -0.3, 0.2, 0.8)): (0.067, -0.0, 0.084),
    ((0.1, 0.3, 0.2, 1.2),): (0.047, 0.16, 0.113),
    ((0.4, 0.0, -0.2, 0.3),): (0.106, -0.005, -0.069),
    ((0.4, 0.0, -0.2, 0.4),): (0.142, -0.017, -0.111),
    ((0.4, 0.0, 0.0, 0.3),): (0.142, 0.008, 0.033),
    ((0.4, 0.0, 0.0, 0.4),): (0.144, 0.001, -0.035),
    ((0.4, 0.0, 0.2, 0.3),): (0.106, 0.02, 0.057),
    ((0.4, 0.0, 0.2, 0.4),): (0.146, 0.018, 0.024),
}

RESPONSE_CARRY: dict[Correction, tuple[float, float, float]] = {
    (): (0.0, 0.0, 0.0),
    ((-0.4, 0.0, -0.2, 0.2),): (-0.073, 0.036, 0.008),
    ((-0.4, 0.0, -0.2, 0.3),): (-0.111, 0.023, -0.012),
    ((-0.4, 0.0, -0.2, 0.4),): (-0.15, 0.031, -0.04),
    ((-0.4, 0.0, -0.2, 0.8),): (-0.238, 0.037, -0.063),
    ((-0.4, 0.0, 0.0, 0.2),): (-0.026, 0.009, 0.029),
    ((-0.4, 0.0, 0.0, 0.3),): (-0.098, 0.018, 0.009),
    ((-0.4, 0.0, 0.0, 0.4),): (-0.131, 0.018, -0.023),
    ((-0.4, 0.0, 0.0, 0.8),): (-0.235, 0.032, 0.01),
    ((-0.4, 0.0, 0.2, 0.2),): (-0.112, 0.042, 0.015),
    ((-0.4, 0.0, 0.2, 0.3),): (-0.112, 0.004, -0.003),
    ((-0.4, 0.0, 0.2, 0.4),): (-0.157, 0.005, -0.017),
    ((-0.4, 0.0, 0.2, 0.8),): (-0.283, 0.007, 0.074),
    ((0.0, -0.3, -0.2, 1.2),): (-0.017, -0.132, -0.113),
    ((0.0, -0.3, 0.2, 1.2),): (-0.025, -0.115, 0.076),
    ((0.0, 0.3, -0.2, 1.2),): (-0.002, -0.0, -0.034),
    ((0.0, 0.3, 0.2, 1.2),): (-0.025, 0.073, 0.116),
    ((0.1, -0.3, -0.2, 0.5),): (0.038, -0.038, -0.048),
    ((0.1, -0.3, -0.2, 0.5), (0.1, 0.3, -0.2, 0.5)): (0.087, 0.07, -0.068),
    ((0.1, -0.3, -0.2, 0.8),): (0.072, -0.087, -0.07),
    ((0.1, -0.3, -0.2, 0.8), (0.1, 0.3, -0.2, 0.8)): (0.146, 0.003, -0.167),
    ((0.1, -0.3, -0.2, 1.2),): (0.093, -0.125, -0.115),
    ((0.1, -0.3, 0.0, 0.5),): (0.001, -0.003, 0.011),
    ((0.1, -0.3, 0.0, 0.8),): (0.082, -0.053, -0.014),
    ((0.1, -0.3, 0.0, 1.2),): (0.087, -0.151, -0.015),
    ((0.1, -0.3, 0.2, 0.5),): (0.011, -0.033, 0.018),
    ((0.1, -0.3, 0.2, 0.5), (0.1, 0.3, 0.2, 0.5)): (0.052, 0.023, 0.091),
    ((0.1, -0.3, 0.2, 0.8),): (0.059, -0.075, 0.061),
    ((0.1, -0.3, 0.2, 0.8), (0.1, 0.3, 0.2, 0.8)): (0.084, -0.0, 0.129),
    ((0.1, -0.3, 0.2, 1.2),): (0.071, -0.128, 0.101),
    ((0.1, 0.3, -0.2, 0.5),): (0.028, 0.046, -0.048),
    ((0.1, 0.3, -0.2, 0.5), (0.1, -0.3, -0.2, 0.5)): (0.021, -0.02, -0.041),
    ((0.1, 0.3, -0.2, 0.8),): (0.028, 0.043, -0.067),
    ((0.1, 0.3, -0.2, 0.8), (0.1, -0.3, -0.2, 0.8)): (0.083, -0.028, -0.152),
    ((0.1, 0.3, -0.2, 1.2),): (-0.001, 0.003, -0.071),
    ((0.1, 0.3, 0.0, 0.5),): (0.015, 0.019, -0.016),
    ((0.1, 0.3, 0.0, 0.8),): (0.027, 0.045, -0.015),
    ((0.1, 0.3, 0.0, 1.2),): (0.066, 0.121, -0.004),
    ((0.1, 0.3, 0.2, 0.5),): (0.03, 0.047, 0.047),
    ((0.1, 0.3, 0.2, 0.5), (0.1, -0.3, 0.2, 0.5)): (0.054, -0.025, 0.065),
    ((0.1, 0.3, 0.2, 0.8),): (0.057, 0.1, 0.079),
    ((0.1, 0.3, 0.2, 0.8), (0.1, -0.3, 0.2, 0.8)): (0.103, 0.043, 0.131),
    ((0.1, 0.3, 0.2, 1.2),): (0.065, 0.177, 0.134),
    ((0.4, 0.0, -0.2, 0.3),): (0.144, -0.002, -0.046),
    ((0.4, 0.0, -0.2, 0.4),): (0.158, -0.017, -0.102),
    ((0.4, 0.0, 0.0, 0.3),): (0.086, 0.01, 0.012),
    ((0.4, 0.0, 0.0, 0.4),): (0.157, 0.001, -0.042),
    ((0.4, 0.0, 0.2, 0.3),): (0.167, 0.031, 0.114),
    ((0.4, 0.0, 0.2, 0.4),): (0.164, 0.016, 0.019),
}

FIRST_WEIGHT = 0.3  # how much the error right after the first correction counts against what the second could reach


def expected(correction, carrying: bool = False) -> tuple[float, float, float]:
    """What the table says `correction` does: (forward m, left m, heading rad)."""
    return (RESPONSE_CARRY if carrying else RESPONSE)[correction]


def plan_correction(error, tol_m: float, tol_rad: float, creep: float, banned=(), carrying: bool = False):
    """The correction, from the measured table, to send now: the first of the two corrections (looking after each) that
    leave the smallest error where the pose will be confirmed. A single best step is not enough: the burst that cancels
    a sideways error also moves the robot forward, and the one that would cancel that moves it sideways again.

    `error` is (forward, left, heading) still to cancel, in the robot's frame. The heading creeps `creep` rad/s to the
    left while the robot waits (to settle, to look, to be confirmed), which the plan includes. The cost of a leftover
    error is its square in tolerances (position radially in `tol_m`, heading in `tol_rad`); a small cost per second of
    command breaks ties. Returns (correction, predicted cost if the pose were confirmed after it).
    """
    table = RESPONSE_CARRY if carrying else RESPONSE
    ex, ey, eyaw = error

    def cost(x, y, w, key):
        return (x * x + y * y) / tol_m**2 + (w / tol_rad) ** 2 + 0.03 * sum(b[3] for b in key)

    best = None
    for first, (fx, fy, fw) in table.items():
        if first in banned:
            continue
        x, y, w = ex - fx, ey - fy, eyaw - fw - creep * SETTLE_LOOK_S  # at the look after it
        confirmed = cost(x, y, w - creep * CONFIRM_DWELL_S, first)
        follow = min(
            cost(x - gx, y - gy, w - gw - creep * (SETTLE_LOOK_S + CONFIRM_DWELL_S), second)
            for second, (gx, gy, gw) in table.items()
        )
        total = FIRST_WEIGHT * confirmed + follow
        if best is None or total < best[2]:
            best = (first, confirmed, total)
    return best[0], best[1]
