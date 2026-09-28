"""
Semi-Active Suspension Controllers for Magnetorheological (MR) Dampers.

Includes:
- Passive Constant Voltage (Soft / Hard)
- Classical Karnopp 2-State On-Off Skyhook Control
- Continuous Linear Skyhook Control
- Groundhook (Tire Road-Holding) Control
- Hybrid Skyhook-Groundhook Control

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
"""

from abc import ABC, abstractmethod
from typing import Dict, Any, Optional
import numpy as np


class BaseSuspensionController(ABC):
    """Abstract base class for semi-active suspension controllers."""

    def __init__(self, name: str, color: str = "#1f77b4"):
        self.name = name
        self.color = color

    @abstractmethod
    def compute_voltage(
        self,
        t: float,
        state: np.ndarray,
        aux: Optional[Dict[str, Any]] = None
    ) -> float:
        """
        Calculate the commanded coil input voltage V_cmd in Volts.

        Args:
            t: Current time [s]
            state: Array of 7 states [z_s, z_s_dot, z_u, z_u_dot, y, z, u]
            aux: Optional auxiliary signals dictionary

        Returns:
            v_cmd: Commanded voltage [V]
        """
        pass

    def reset(self) -> None:
        """Reset internal controller states if any."""
        pass


class PassiveController(BaseSuspensionController):
    """Fixed-voltage passive damper emulation (e.g. 0.0 V Soft, 2.0 V Hard)."""

    def __init__(self, voltage: float = 0.0, name: Optional[str] = None, color: str = "#1f77b4"):
        if name is None:
            name = f"Passive ({voltage:.1f} V)"
        super().__init__(name=name, color=color)
        self.voltage = float(voltage)

    def compute_voltage(self, t: float, state: np.ndarray, aux: Optional[Dict[str, Any]] = None) -> float:
        return self.voltage


class BasicPassiveDamperController(BaseSuspensionController):
    """
    Classical Basic Linear Passive Shock Absorber.
    Generates standard viscous damping force F_d = c_s * (z_s_dot - z_u_dot)
    without MR fluid hysteresis, magnetic field coil dynamics, or gas pre-charge.
    Default c_s = 1500 N*s/m corresponds to typical passenger car damping ratio (zeta ~ 0.28).
    """

    def __init__(
        self,
        c_s: float = 1500.0,
        name: str = "Basic Passive (Linear)",
        color: str = "#7f7f7f"
    ):
        super().__init__(name=name, color=color)
        self.c_s = float(c_s)
        self.is_linear_passive = True

    def compute_voltage(self, t: float, state: np.ndarray, aux: Optional[Dict[str, Any]] = None) -> float:
        return 0.0


class SkyhookController(BaseSuspensionController):
    """
    Karnopp Skyhook Semi-Active Controller.

    Modulations:
    - 'two_state': Classical 2-State (On/Off) switching:
          If z_s_dot * (z_s_dot - z_u_dot) >= 0 -> V_max
          Else -> V_min
    - 'continuous': Modulates voltage proportionally to ideal skyhook force:
          F_sky = c_sky * z_s_dot
          If z_s_dot * (z_s_dot - z_u_dot) >= 0 -> V_min + (F_sky / F_max) * (V_max - V_min)
          Else -> V_min
    """

    def __init__(
        self,
        mode: str = "two_state",
        v_min: float = 0.0,
        v_max: float = 2.0,
        c_sky: float = 2500.0,
        f_max: float = 2000.0,
        name: str = "Skyhook Semi-Active",
        color: str = "#2ca02c"
    ):
        super().__init__(name=name, color=color)
        self.mode = mode.lower()
        self.v_min = v_min
        self.v_max = v_max
        self.c_sky = c_sky
        self.f_max = f_max

    def compute_voltage(self, t: float, state: np.ndarray, aux: Optional[Dict[str, Any]] = None) -> float:
        z_s_dot = state[1]
        z_u_dot = state[3]
        v_rel = z_s_dot - z_u_dot  # x_dot

        # Skyhook switching criterion: Product of absolute body velocity and relative velocity
        condition = z_s_dot * v_rel

        if self.mode == "two_state":
            return self.v_max if condition >= 0.0 else self.v_min
        elif self.mode == "continuous":
            if condition >= 0.0:
                f_sky = self.c_sky * abs(z_s_dot)
                v_scaled = self.v_min + (f_sky / self.f_max) * (self.v_max - self.v_min)
                return float(np.clip(v_scaled, self.v_min, self.v_max))
            else:
                return self.v_min
        else:
            raise ValueError(f"Unknown Skyhook mode: {self.mode}")


class GroundhookController(BaseSuspensionController):
    """
    Groundhook Semi-Active Controller focused on Unsprung Mass damping and Tire Road-Holding.

    Logic:
        If -z_u_dot * (z_s_dot - z_u_dot) >= 0 -> V_max
        Else -> V_min
    """

    def __init__(
        self,
        v_min: float = 0.0,
        v_max: float = 2.0,
        name: str = "Groundhook Semi-Active",
        color: str = "#ff7f0e"
    ):
        super().__init__(name=name, color=color)
        self.v_min = v_min
        self.v_max = v_max

    def compute_voltage(self, t: float, state: np.ndarray, aux: Optional[Dict[str, Any]] = None) -> float:
        z_s_dot = state[1]
        z_u_dot = state[3]
        v_rel = z_s_dot - z_u_dot

        # Groundhook criterion
        condition = -z_u_dot * v_rel
        return self.v_max if condition >= 0.0 else self.v_min


class HybridSkyhookGroundhookController(BaseSuspensionController):
    """
    Hybrid Skyhook-Groundhook Controller.
    Balances passenger ride comfort (Skyhook) with tire road-holding safety (Groundhook).

    Parameter alpha in [0.0, 1.0]:
        alpha = 1.0 -> Pure Skyhook (Body comfort optimized)
        alpha = 0.0 -> Pure Groundhook (Tire force variation minimized)
        alpha = 0.5 - 0.7 -> Balanced tradeoff
    """

    def __init__(
        self,
        alpha: float = 0.65,
        v_min: float = 0.0,
        v_max: float = 2.0,
        name: str = "Hybrid Skyhook-Groundhook",
        color: str = "#9467bd"
    ):
        super().__init__(name=name, color=color)
        self.alpha = float(np.clip(alpha, 0.0, 1.0))
        self.v_min = v_min
        self.v_max = v_max

    def compute_voltage(self, t: float, state: np.ndarray, aux: Optional[Dict[str, Any]] = None) -> float:
        z_s_dot = state[1]
        z_u_dot = state[3]
        v_rel = z_s_dot - z_u_dot

        sigma = self.alpha * z_s_dot - (1.0 - self.alpha) * z_u_dot
        condition = sigma * v_rel
        return self.v_max if condition >= 0.0 else self.v_min
