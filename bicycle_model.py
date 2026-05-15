"""
4WS bicycle dynamics model and simulation utilities.

Equations from doc/01_bicycle_model.md (section 14/18).
Equivalent front-wheel angle from doc/05_equivalent_front_steer.md.
"""

import numpy as np
from dataclasses import dataclass
from scipy.integrate import solve_ivp


@dataclass
class VehicleParams:
    m: float = 1500.0
    Iz: float = 2500.0
    lf: float = 1.2
    lr: float = 1.4
    Cf: float = 80000.0
    Cr: float = 80000.0

    @property
    def L(self):
        return self.lf + self.lr


def bicycle_ode(t, state, vx, delta_f_func, delta_r_func, params):
    """
    4WS bicycle model ODE.

    State: [vy, r, psi, X, Y]
    """
    vy, r, psi, X, Y = state
    df = delta_f_func(t)
    dr = delta_r_func(t)

    m, Iz, lf, lr = params.m, params.Iz, params.lf, params.lr
    Cf, Cr = params.Cf, params.Cr

    alpha_f = df - (vy + lf * r) / vx
    alpha_r = dr - (vy - lr * r) / vx
    Fyf = Cf * alpha_f
    Fyr = Cr * alpha_r

    dvy = (Fyf + Fyr) / m - vx * r
    dr_dt = (lf * Fyf - lr * Fyr) / Iz
    dpsi = r
    dX = vx * np.cos(psi) - vy * np.sin(psi)
    dY = vx * np.sin(psi) + vy * np.cos(psi)

    return [dvy, dr_dt, dpsi, dX, dY]


def simulate(vx, delta_f_func, delta_r_func, params, t_span, dt=0.005):
    t_eval = np.arange(t_span[0], t_span[1], dt)
    sol = solve_ivp(
        bicycle_ode,
        t_span,
        [0.0, 0.0, 0.0, 0.0, 0.0],
        args=(vx, delta_f_func, delta_r_func, params),
        t_eval=t_eval,
        method="RK45",
        rtol=1e-8,
        atol=1e-10,
    )
    t = sol.t
    vy, r, psi, X, Y = sol.y
    beta = np.arctan2(vy, vx)
    return t, vy, r, psi, X, Y, beta


def kinematic_equivalent(delta_f, delta_r, params=None):
    return delta_f - delta_r


def force_equivalent(delta_f, delta_r, params):
    return delta_f + (params.Cr / params.Cf) * delta_r


def moment_equivalent(delta_f, delta_r, params):
    return delta_f - (params.lr * params.Cr) / (params.lf * params.Cf) * delta_r


def const_steer(value):
    return lambda t: value


def default_params():
    return VehicleParams()
