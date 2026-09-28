"""
Spencer Modified Bouc-Wen Magnetorheological (MR) Damper Model.

Author:
    W. M. Baig

Citation Request:
    If you use this model or code in your research, please cite:
    [1] Z. Yu, R. Luo, P. Wu, W. M. Baig, H. Ma, and Z. Hou, "Robust finite-frequency vibration
        control of in-wheel motor driving vehicles based on torque coordination and motor
        suspension," IEEE Transactions on Transportation Electrification, 2025.
        doi: 10.1109/TTE.2025.3535765
    [2] W. M. Baig, Z. Yu, H. Ma, and Z. Hou, "Adaptive vibration control of in-wheel motor
        drive vehicles with preview information," in Proc. IEEE 101st Vehicular Technology
        Conference (VTC2025-Spring), Oslo, Norway, 2025.
        doi: 10.1109/VTC2025-Spring65109.2025.11174543
    [3] W. M. Baig, Z. Hou, and S. Ijaz, "Fractional order controller design for a semi-active
        suspension system using Nelder-Mead optimization," in Proc. 29th Chinese Control and
        Decision Conference (CCDC), 2017, pp. 2808-2813.

All internal calculations adhere strictly to SI base units (m, s, N, V, rad).
"""

from typing import Tuple, Optional, Callable, Dict, Any
import numpy as np
from scipy.integrate import solve_ivp

from quarter_car.parameters import MRDamperParameters


class SpencerModifiedBoucWenMRDamper:
    """
    Spencer Modified Bouc-Wen phenomenological model for Magnetorheological dampers.

    Internal Damper State Vector:
        state = [y, z, u]
            y: Internal float plate displacement [m]
            z: Evolutionary hysteretic state variable [m]
            u: Effective filtered magnetic coil voltage [V]
    """

    def __init__(self, params: Optional[MRDamperParameters] = None):
        self.params = params if params is not None else MRDamperParameters()
        self.state = np.zeros(3, dtype=np.float64)

    def reset(self, initial_state: Optional[np.ndarray] = None) -> None:
        """Reset internal damper states [y, z, u]."""
        if initial_state is not None:
            self.state = np.array(initial_state, dtype=np.float64)
        else:
            self.state = np.zeros(3, dtype=np.float64)

    def compute_derivatives(
        self,
        t: float,
        state: np.ndarray,
        x: float,
        x_dot: float,
        v_cmd: float
    ) -> Tuple[np.ndarray, float]:
        """
        Compute damper state derivatives and instantaneous damping force.

        Args:
            t: Current time [s]
            state: Array-like containing [y, z, u]
            x: Piston rod displacement [m] (extension positive)
            x_dot: Piston rod velocity [m/s]
            v_cmd: Commanded coil input voltage [V]

        Returns:
            derivs: np.ndarray [dy/dt, dz/dt, du/dt]
            f_mr: Total output damper force [N]
        """
        p = self.params
        y, z, u = state[0], state[1], state[2]

        # 1. Voltage saturation limit and 1st-order coil inductance delay
        v_clamped = np.clip(v_cmd, p.v_min, p.v_max)
        u_dot = -p.eta * (u - v_clamped)

        # 2. Voltage-dependent physical parameters
        alpha = p.alpha_a + p.alpha_b * u
        c0 = p.c0_a + p.c0_b * u
        c1 = p.c1_a + p.c1_b * u

        # 3. Intermediate node velocity (dy/dt) from force equilibrium across plate y:
        #    (c0 + c1) * y_dot = alpha * z + c0 * x_dot + k0 * (x - y)
        denom = c0 + c1
        y_dot = (alpha * z + c0 * x_dot + p.k0 * (x - y)) / denom

        # 4. Hysteretic evolutionary rate (dz/dt)
        # Numerical safeguard against unphysical divergence in explicit solvers (z_max = 5.14 mm)
        z_safe = float(np.clip(z, -0.015, 0.015))
        vel_rel = x_dot - y_dot
        abs_z = abs(z_safe)
        z_n = abs_z ** p.n
        z_n_minus_1 = abs_z ** (p.n - 1.0) if abs_z > 1e-18 else 0.0

        z_dot = (
            p.A * vel_rel
            - p.beta * vel_rel * z_n
            - p.gamma * abs(vel_rel) * z_n_minus_1 * z_safe
        )

        # 5. Resultant damping force transmitted to chassis body:
        #    F_MR = c1 * y_dot + k1 * (x - x0)
        f_mr = c1 * y_dot + p.k1 * (x - p.x0)

        derivs = np.array([y_dot, z_dot, u_dot], dtype=np.float64)
        return derivs, f_mr

    def compute_force(self, state: np.ndarray, x: float, x_dot: float) -> float:
        """
        Compute output damping force directly from current states and kinematics.
        """
        p = self.params
        y, z, u = state[0], state[1], state[2]
        c0 = p.c0_a + p.c0_b * u
        c1 = p.c1_a + p.c1_b * u
        alpha = p.alpha_a + p.alpha_b * u
        y_dot = (alpha * z + c0 * x_dot + p.k0 * (x - y)) / (c0 + c1)
        return c1 * y_dot + p.k1 * (x - p.x0)

    def simulate_trajectory(
        self,
        t_span: Tuple[float, float],
        disp_func: Callable[[float], float],
        vel_func: Callable[[float], float],
        volt_func: Callable[[float], float],
        dt: float = 1e-4,
        initial_state: Optional[np.ndarray] = None
    ) -> Dict[str, np.ndarray]:
        """
        Simulate isolated MR damper response under prescribed kinematic motion.
        """
        if initial_state is None:
            initial_state = np.zeros(3, dtype=np.float64)

        t_eval = np.linspace(t_span[0], t_span[1], max(2, int(np.round((t_span[1] - t_span[0]) / dt)) + 1))

        def ode_func(t, state):
            x = disp_func(t)
            x_dot = vel_func(t)
            v = volt_func(t)
            derivs, _ = self.compute_derivatives(t, state, x, x_dot, v)
            return derivs

        sol = solve_ivp(
            ode_func,
            t_span,
            initial_state,
            t_eval=t_eval,
            method='Radau',
            rtol=1e-6,
            atol=1e-9
        )

        y = sol.y[0]
        z = sol.y[1]
        u = sol.y[2]
        t = sol.t

        x = np.array([disp_func(ti) for ti in t])
        x_dot = np.array([vel_func(ti) for ti in t])

        # Output force vector
        forces = np.zeros_like(t)
        for i in range(len(t)):
            forces[i] = self.compute_force(np.array([y[i], z[i], u[i]]), x[i], x_dot[i])

        return {
            'time': t,
            'displacement': x,
            'velocity': x_dot,
            'force': forces,
            'y': y,
            'z': z,
            'u': u
        }
