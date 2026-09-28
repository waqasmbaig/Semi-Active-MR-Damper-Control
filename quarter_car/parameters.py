"""
Physical Parameters for Quarter-Car Model and Spencer Modified Bouc-Wen MR Damper.

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

All internal calculations adhere strictly to SI base units (m, kg, s, N, V, rad).
"""

from dataclasses import dataclass
import numpy as np


@dataclass
class MRDamperParameters:
    """
    Physical parameters for Spencer's Modified Bouc-Wen Magnetorheological Damper.
    Calibrated against experimental MTS dyno data across 0.0 - 2.0 V commands.
    """
    # Viscous damping parameters
    c0_a: float = 784.0        # Zero-voltage viscous damping coefficient [N*s/m]
    c0_b: float = 1803.0       # Field-dependent viscous damping gain [N*s/(m*V)]
    
    # Dashpot stiffness
    k0: float = 3610.0         # Post-yield stiffness [N/m]
    
    # Gas accumulator parameters
    c1_a: float = 14649.0      # Gas accumulator damping coefficient [N*s/m]
    c1_b: float = 34622.0      # Field-dependent accumulator damping gain [N*s/(m*V)]
    k1: float = 840.0          # Gas accumulator stiffness [N/m]
    x0: float = 0.0245         # Initial accumulator displacement offset [m] (24.5 mm)
    
    # Bouc-Wen hysteresis parameters
    alpha_a: float = 12441.0   # Base hysteretic force coefficient [N/m]
    alpha_b: float = 38430.0   # Field-dependent hysteretic force gain [N/(m*V)]
    gamma: float = 136320.0    # Hysteresis shape parameter [m^-2]
    beta: float = 2059020.0    # Hysteresis shape parameter [m^-2]
    A: float = 58.0            # Hysteresis restoring scale factor [-]
    n: float = 2.0             # Smoothness exponent of yield transition [-]
    
    # Electromagnetic coil dynamics
    eta: float = 190.0         # Coil first-order time response rate [s^-1] (tau = 1/eta ~ 5.26 ms)
    v_min: float = 0.0         # Minimum coil input voltage [V]
    v_max: float = 2.0         # Maximum coil saturation voltage [V]


@dataclass
class QuarterCarParameters:
    """
    Physical parameters for the 2-Degree-of-Freedom (2-DOF) Quarter-Car Suspension.
    Adheres strictly to ISO 8855 conventions (positive vertical z is upwards).
    """
    m_s: float = 320.0         # Sprung mass (chassis body quarter) [kg]
    m_u: float = 40.0          # Unsprung mass (wheel, hub, tire, brake assembly) [kg]
    k_s: float = 22000.0       # Suspension primary coil spring stiffness [N/m]
    c_s_passive: float = 0.0   # Parallel passive viscous damping (0 when purely MRD) [N*s/m]
    c_s_linear: float = 1500.0 # Standard basic linear passive shock absorber damping [N*s/m]
    k_t: float = 190000.0      # Tire radial vertical stiffness [N/m]
    c_t: float = 0.0           # Tire vertical damping coefficient [N*s/m]
    g: float = 9.80665         # Gravitational acceleration [m/s^2]

    @property
    def sprung_natural_frequency_hz(self) -> float:
        """Undamped natural frequency of the sprung mass (body bounce mode) in Hz."""
        return (1.0 / (2.0 * np.pi)) * np.sqrt(self.k_s / self.m_s)

    @property
    def unsprung_natural_frequency_hz(self) -> float:
        """Undamped natural frequency of the unsprung mass (wheel hop mode) in Hz."""
        return (1.0 / (2.0 * np.pi)) * np.sqrt((self.k_t + self.k_s) / self.m_u)

    @property
    def critical_damping(self) -> float:
        """Critical damping coefficient of the sprung mass bounce mode [N*s/m]."""
        return 2.0 * np.sqrt(self.m_s * self.k_s)

    @property
    def linear_damping_ratio(self) -> float:
        """Damping ratio zeta of the basic linear passive damper [-]."""
        return self.c_s_linear / self.critical_damping

    @property
    def static_suspension_deflection(self) -> float:
        """Static suspension equilibrium deflection [m]."""
        return (self.m_s * self.g) / self.k_s

    @property
    def static_tire_deflection(self) -> float:
        """Static tire equilibrium deflection [m]."""
        return ((self.m_s + self.m_u) * self.g) / self.k_t
