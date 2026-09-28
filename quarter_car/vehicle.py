"""
Two-Degree-of-Freedom (2-DOF) Quarter-Car Vehicle Suspension Model.

Author:
    W. M. Baig

Coordinate System & Formulations:
    Adheres strictly to ISO 8855 conventions (vertical axis z positive upwards):
    - z_r: Road excitation surface vertical profile [m]
    - z_u: Unsprung mass displacement [m]
    - z_s: Sprung mass displacement [m]
    - x = z_s - z_u: Suspension deflection [m] (extension positive)

    (a) Passive Quarter-Car System:
        m_s * z_s_ddot + c_s * (z_s_dot - z_u_dot) + k_s * (z_s - z_u) = 0
        m_u * z_u_ddot - c_s * (z_s_dot - z_u_dot) - k_s * (z_s - z_u) + k_t * (z_u - z_r) = 0

    (b) Semi-Active Quarter-Car System:
        m_s * z_s_ddot + k_s * (z_s - z_u) + F_d = 0
        m_u * z_u_ddot - k_s * (z_s - z_u) - F_d + k_t * (z_u - z_r) = 0
        where F_d is the controllable damping force delivered by the MR damper (Spencer MBW).

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
"""

from typing import Tuple, Optional, Dict, Any
import numpy as np

from quarter_car.parameters import QuarterCarParameters, MRDamperParameters
from quarter_car.mr_damper import SpencerModifiedBoucWenMRDamper


class QuarterCarModel:
    """
    2-DOF Quarter-Car model dynamically coupled with the Spencer Modified Bouc-Wen MR Damper.

    Complete State Vector (7 States):
        state = [z_s, z_s_dot, z_u, z_u_dot, y, z, u]
            z_s:     Sprung mass vertical displacement [m]
            z_s_dot: Sprung mass vertical velocity [m/s]
            z_u:     Unsprung mass vertical displacement [m]
            z_u_dot: Unsprung mass vertical velocity [m/s]
            y:       Damper intermediate node displacement [m]
            z:       Damper Bouc-Wen hysteretic displacement [m]
            u:       Effective filtered coil voltage [V]
    """

    STATE_NAMES = ["z_s", "z_s_dot", "z_u", "z_u_dot", "y", "z", "u"]

    def __init__(
        self,
        vehicle_params: Optional[QuarterCarParameters] = None,
        damper_params: Optional[MRDamperParameters] = None
    ):
        self.vehicle_params = vehicle_params if vehicle_params is not None else QuarterCarParameters()
        self.damper = SpencerModifiedBoucWenMRDamper(damper_params)

    def equations_of_motion(
        self,
        t: float,
        state: np.ndarray,
        z_r: float,
        z_r_dot: float,
        v_cmd: float,
        use_linear_damper: bool = False,
        c_s_linear: Optional[float] = None
    ) -> Tuple[np.ndarray, Dict[str, float]]:
        """
        Evaluate closed-loop state derivatives and auxiliary mechanical variables.

        Args:
            t: Current time [s]
            state: Array of 7 states [z_s, z_s_dot, z_u, z_u_dot, y, z, u]
            z_r: Road elevation [m]
            z_r_dot: Road vertical velocity [m/s]
            v_cmd: Commanded MR damper control voltage [V]
            use_linear_damper: If True, evaluates standard linear shock absorber (c_s * x_dot)
            c_s_linear: Optional linear damping coefficient [N*s/m] (defaults to params.c_s_linear)

        Returns:
            derivs: np.ndarray of shape (7,) representing d(state)/dt
            aux: Dictionary of auxiliary signals (forces, accelerations, deflections)
        """
        vp = self.vehicle_params
        z_s, z_s_dot, z_u, z_u_dot, y_d, z_d, u_d = state

        # Suspension relative kinematics (extension positive)
        susp_deflection = z_s - z_u
        susp_velocity = z_s_dot - z_u_dot

        # Tire radial deflection and rate
        tire_deflection = z_u - z_r
        tire_velocity = z_u_dot - z_r_dot

        if use_linear_damper:
            # Basic linear passive shock absorber: F_d = c_s * x_dot
            c_val = c_s_linear if c_s_linear is not None else vp.c_s_linear
            f_mr = c_val * susp_velocity
            mr_derivs = np.zeros(3, dtype=np.float64)
        else:
            # Spencer Modified Bouc-Wen MR Damper derivatives and instantaneous force
            damper_state = np.array([y_d, z_d, u_d], dtype=np.float64)
            mr_derivs, f_mr = self.damper.compute_derivatives(
                t, damper_state, susp_deflection, susp_velocity, v_cmd
            )

        # Mechanical Forces
        f_spring = vp.k_s * susp_deflection + vp.c_s_passive * susp_velocity
        f_tire = vp.k_t * tire_deflection + vp.c_t * tire_velocity
        f_suspension_total = f_spring + f_mr

        # Accelerations from Newton's Second Law
        z_s_ddot = (-f_suspension_total) / vp.m_s
        z_u_ddot = (f_suspension_total - f_tire) / vp.m_u

        derivs = np.array([
            z_s_dot,
            z_s_ddot,
            z_u_dot,
            z_u_ddot,
            mr_derivs[0],  # dy/dt
            mr_derivs[1],  # dz/dt
            mr_derivs[2],  # du/dt
        ], dtype=np.float64)

        aux = {
            "f_mr": f_mr,
            "f_spring": f_spring,
            "f_tire": f_tire,
            "f_susp_total": f_suspension_total,
            "susp_deflection": susp_deflection,
            "susp_velocity": susp_velocity,
            "tire_deflection": tire_deflection,
            "z_s_ddot": z_s_ddot,
            "z_u_ddot": z_u_ddot,
            "v_cmd": v_cmd,
            "u_effective": u_d,
            "power_dissipated": f_mr * susp_velocity
        }

        return derivs, aux
