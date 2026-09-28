"""
Standard Road Elevation Profiles and Road-Tire Excitation Models.

Includes:
- Discrete ISO Obstacle Bump (Haversine profile with analytical derivative)
- ISO 8608 Stochastic Road Profile (Class A, B, C, D, E) via state-space white noise filtering
- Swept-Frequency Chirp Excitation (for transmissibility Bode/frequency analysis)
- Smoothed Step Excitation (curb strike)

Author:
    W. M. Baig
"""

from abc import ABC, abstractmethod
from typing import Tuple
import numpy as np


class BaseRoadProfile(ABC):
    """Abstract base class for vehicle vertical road excitations."""

    @abstractmethod
    def evaluate(self, t: float) -> Tuple[float, float]:
        """
        Evaluate road elevation z_r and road vertical velocity z_r_dot at time t.

        Returns:
            z_r: Road surface vertical displacement [m]
            z_r_dot: Road surface vertical velocity [m/s]
        """
        pass


class HaversineBumpRoad(BaseRoadProfile):
    """
    Standard ISO discrete obstacle bump excitation (Haversine bump).

    Formula:
        z_r(t) = 0.5 * H * (1 - cos(2*pi*tau)) for 0 <= tau <= 1
        where tau = (t - t_start) / t_bump, and t_bump = L / v_vehicle
    """

    def __init__(
        self,
        height: float = 0.05,       # 50 mm bump [m]
        length: float = 1.0,        # 1.0 m bump length [m]
        velocity_kmh: float = 45.0, # Vehicle forward speed [km/h]
        t_start: float = 0.5        # Start time of bump [s]
    ):
        self.height = float(height)
        self.length = float(length)
        self.v_mps = float(velocity_kmh) / 3.6
        self.t_start = float(t_start)
        self.t_bump = self.length / self.v_mps
        self.t_end_bump = self.t_start + self.t_bump

    def evaluate(self, t: float) -> Tuple[float, float]:
        if self.t_start <= t <= self.t_end_bump:
            tau = (t - self.t_start) / self.t_bump
            zr = 0.5 * self.height * (1.0 - np.cos(2.0 * np.pi * tau))
            zr_dot = 0.5 * self.height * (2.0 * np.pi / self.t_bump) * np.sin(2.0 * np.pi * tau)
            return zr, zr_dot
        return 0.0, 0.0


class ISO8608RandomRoad(BaseRoadProfile):
    """
    Stochastic Road Roughness modeled in accordance with ISO 8608.
    Implemented as a pre-computed or interpolated time-series generated from
    first-order filtered white noise in SI units:
        d(z_r)/dt = -2*pi*f_low * z_r + 2*pi * n_0 * sqrt(G_q(n_0) * v) * w(t)
    """

    # Geometric mean roughness coefficients G_q(n_0) at n_0 = 0.1 cycles/m [m^3]
    ROUGHNESS_COEFFS = {
        "A": 16e-6,   # Very good (Motorway / Interstate)
        "B": 64e-6,   # Good (Principal road)
        "C": 256e-6,  # Average (Major road with imperfections)
        "D": 1024e-6, # Poor (Minor road)
        "E": 4096e-6  # Very poor (Unpaved track)
    }

    def __init__(
        self,
        road_class: str = "C",
        velocity_kmh: float = 50.0,
        duration: float = 5.0,
        dt: float = 1e-4,
        seed: int = 42,
        f_low: float = 0.1  # Low-frequency cutoff [Hz]
    ):
        self.road_class = road_class.upper()
        self.v_mps = float(velocity_kmh) / 3.6
        self.duration = float(duration)
        self.dt = float(dt)
        self.seed = seed
        self.f_low = f_low

        g0 = self.ROUGHNESS_COEFFS.get(self.road_class, 256e-6)
        n0 = 0.1  # [cycles/m]

        num_steps = int(np.ceil(self.duration / self.dt)) + 1
        self.t_arr = np.linspace(0.0, self.duration, num_steps)
        self.zr_arr = np.zeros(num_steps)
        self.zr_dot_arr = np.zeros(num_steps)

        # Generate deterministic synthetic road profile
        rng = np.random.default_rng(self.seed)
        white_noise = rng.normal(0.0, 1.0 / np.sqrt(self.dt), size=num_steps)

        sigma_coeff = 2.0 * np.pi * n0 * np.sqrt(g0 * self.v_mps)
        omega_low = 2.0 * np.pi * self.f_low

        zr = 0.0
        for i in range(num_steps):
            w = white_noise[i]
            zr_dot = -omega_low * zr + sigma_coeff * w
            zr += zr_dot * self.dt
            self.zr_arr[i] = zr
            self.zr_dot_arr[i] = zr_dot

    def evaluate(self, t: float) -> Tuple[float, float]:
        if t <= 0.0:
            return float(self.zr_arr[0]), float(self.zr_dot_arr[0])
        if t >= self.duration:
            return float(self.zr_arr[-1]), float(self.zr_dot_arr[-1])
        # Fast linear interpolation
        idx = int(t / self.dt)
        if idx >= len(self.zr_arr) - 1:
            return float(self.zr_arr[-1]), float(self.zr_dot_arr[-1])
        frac = (t - self.t_arr[idx]) / self.dt
        zr = self.zr_arr[idx] + frac * (self.zr_arr[idx + 1] - self.zr_arr[idx])
        zr_dot = self.zr_dot_arr[idx] + frac * (self.zr_dot_arr[idx + 1] - self.zr_dot_arr[idx])
        return float(zr), float(zr_dot)


class ChirpHarmonicRoad(BaseRoadProfile):
    """
    Linear frequency sweep (chirp) for evaluating transmissibility across frequencies.
    """

    def __init__(
        self,
        f_start: float = 0.5,
        f_end: float = 15.0,
        amplitude: float = 0.01,  # 10 mm amplitude [m]
        duration: float = 10.0
    ):
        self.f_start = f_start
        self.f_end = f_end
        self.amplitude = amplitude
        self.duration = duration
        self.beta = (f_end - f_start) / duration

    def evaluate(self, t: float) -> Tuple[float, float]:
        if t < 0.0:
            return 0.0, 0.0
        t_c = min(t, self.duration)
        phase = 2.0 * np.pi * (self.f_start * t_c + 0.5 * self.beta * (t_c ** 2))
        f_inst = self.f_start + self.beta * t_c
        omega_inst = 2.0 * np.pi * f_inst

        zr = self.amplitude * np.sin(phase)
        zr_dot = self.amplitude * omega_inst * np.cos(phase)
        return float(zr), float(zr_dot)


class StepRoad(BaseRoadProfile):
    """Smoothed step displacement input (curb strike)."""

    def __init__(self, step_height: float = 0.03, t_step: float = 0.5, rise_time: float = 0.02):
        self.step_height = step_height
        self.t_step = t_step
        self.rise_time = rise_time

    def evaluate(self, t: float) -> Tuple[float, float]:
        if t < self.t_step:
            return 0.0, 0.0
        elif t <= self.t_step + self.rise_time:
            tau = (t - self.t_step) / self.rise_time
            # Cubic smooth step (zero derivative at boundaries)
            s = 3.0 * (tau ** 2) - 2.0 * (tau ** 3)
            s_dot = (6.0 * tau - 6.0 * (tau ** 2)) / self.rise_time
            return self.step_height * s, self.step_height * s_dot
        else:
            return self.step_height, 0.0
