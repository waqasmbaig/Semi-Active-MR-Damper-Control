"""
Simulation Engine for 2-DOF Quarter-Car Semi-Active Suspension with MR Damper.

Supports both implicit stiff adaptive solvers (SciPy Radau / BDF) and
high-performance deterministic fixed-step Runge-Kutta 4th Order (RK4).

Author:
    W. M. Baig
"""

from typing import Tuple, Optional, Dict, Any, List
import numpy as np
from scipy.integrate import solve_ivp

from quarter_car.parameters import QuarterCarParameters, MRDamperParameters
from quarter_car.vehicle import QuarterCarModel
from quarter_car.controllers import BaseSuspensionController, PassiveController
from quarter_car.road_profiles import BaseRoadProfile, HaversineBumpRoad
from quarter_car.metrics import compute_vibration_metrics


class SimulationResult:
    """Container holding time-series trajectories, signals, and scalar performance metrics."""

    def __init__(
        self,
        time: np.ndarray,
        states: np.ndarray,
        road_elevation: np.ndarray,
        road_velocity: np.ndarray,
        f_mr: np.ndarray,
        v_cmd: np.ndarray,
        u_effective: np.ndarray,
        z_s_ddot: np.ndarray,
        z_u_ddot: np.ndarray,
        susp_deflection: np.ndarray,
        susp_velocity: np.ndarray,
        tire_deflection: np.ndarray,
        f_tire: np.ndarray,
        power: np.ndarray,
        controller_name: str,
        color: str
    ):
        self.time = time
        self.states = states
        self.z_s = states[0]
        self.z_s_dot = states[1]
        self.z_u = states[2]
        self.z_u_dot = states[3]
        self.y = states[4]
        self.z = states[5]
        self.u = states[6]
        self.road_elevation = road_elevation
        self.road_velocity = road_velocity
        self.f_mr = f_mr
        self.v_cmd = v_cmd
        self.u_effective = u_effective
        self.z_s_ddot = z_s_ddot
        self.z_u_ddot = z_u_ddot
        self.susp_deflection = susp_deflection
        self.susp_velocity = susp_velocity
        self.tire_deflection = tire_deflection
        self.f_tire = f_tire
        self.power = power
        self.controller_name = controller_name
        self.color = color

        self.metrics = compute_vibration_metrics(
            time=self.time,
            z_s=self.z_s,
            z_s_ddot=self.z_s_ddot,
            susp_deflection=self.susp_deflection,
            tire_deflection=self.tire_deflection,
            f_mr=self.f_mr,
            power=self.power
        )

    def summary(self) -> str:
        """Return formatted string summary of performance indicators."""
        m = self.metrics
        return (
            f"[{self.controller_name}]\n"
            f"  RMS Body Accel:        {m['rms_accel_mps2']:6.3f} m/s^2\n"
            f"  Peak Body Disp:        {m['peak_disp_mm']:6.2f} mm\n"
            f"  Settling Time (5%):    {m['settling_time_s']:6.3f} s\n"
            f"  Peak Susp Deflection:  {m['peak_susp_travel_mm']:6.2f} mm\n"
            f"  RMS Tire Deflection:   {m['rms_tire_deflection_mm']:6.2f} mm\n"
            f"  Peak MR Damper Force:  {m['peak_mr_force_N']:6.1f} N\n"
            f"  Crest Factor:          {m['crest_factor']:6.2f}"
        )


class QuarterCarSimulator:
    """
    Simulation executive orchestrating vehicle model, MR damper, road profile, and controller.
    """

    def __init__(
        self,
        vehicle_model: Optional[QuarterCarModel] = None,
        road_profile: Optional[BaseRoadProfile] = None
    ):
        self.model = vehicle_model if vehicle_model is not None else QuarterCarModel()
        self.road = road_profile if road_profile is not None else HaversineBumpRoad()

    def simulate(
        self,
        controller: BaseSuspensionController,
        t_span: Tuple[float, float] = (0.0, 2.5),
        dt: float = 2e-4,
        solver: str = "rk4",
        initial_state: Optional[np.ndarray] = None
    ) -> SimulationResult:
        """
        Execute closed-loop simulation over the specified time span.

        Args:
            controller: Instance of BaseSuspensionController
            t_span: (t_start, t_end) in seconds
            dt: Fixed evaluation / solver time step [s]
            solver: 'radau' (implicit stiff SciPy), 'bdf', or 'rk4' (fixed-step explicit)
            initial_state: Initial 7-element state array (default all zeros)

        Returns:
            SimulationResult object
        """
        controller.reset()
        t_start, t_end = t_span
        num_steps = int(np.round((t_end - t_start) / dt)) + 1
        t_eval = np.linspace(t_start, t_end, num_steps)

        if initial_state is None:
            init_state = np.zeros(7, dtype=np.float64)
        else:
            init_state = np.array(initial_state, dtype=np.float64)

        if solver.lower() == "rk4":
            return self._simulate_rk4(controller, t_eval, dt, init_state)
        else:
            return self._simulate_scipy(controller, t_span, t_eval, solver.lower(), init_state)

    def _simulate_scipy(
        self,
        controller: BaseSuspensionController,
        t_span: Tuple[float, float],
        t_eval: np.ndarray,
        method: str,
        init_state: np.ndarray
    ) -> SimulationResult:
        method_name = "Radau" if method == "radau" else "BDF"

        def ode_func(t, state):
            zr, zr_dot = self.road.evaluate(t)
            v_cmd = controller.compute_voltage(t, state)
            derivs, _ = self.model.equations_of_motion(t, state, zr, zr_dot, v_cmd)
            return derivs

        sol = solve_ivp(
            ode_func,
            t_span,
            init_state,
            t_eval=t_eval,
            method=method_name,
            rtol=1e-5,
            atol=1e-8
        )

        return self._post_process(sol.t, sol.y, controller)

    def _simulate_rk4(
        self,
        controller: BaseSuspensionController,
        t_eval: np.ndarray,
        dt: float,
        init_state: np.ndarray
    ) -> SimulationResult:
        n_points = len(t_eval)
        states = np.zeros((7, n_points), dtype=np.float64)
        states[:, 0] = init_state

        curr_state = init_state.copy()

        # To ensure numerical stability for stiff Bouc-Wen dynamics (eigenvalues ~ 10,000 s^-1),
        # limit internal RK4 sub-step size to <= 5e-5 s (ISO SIL stability guideline)
        max_substep = 5e-5
        substeps = max(1, int(np.ceil(dt / max_substep)))
        h = dt / substeps

        def f_sys(t, st):
            zr, zr_dot = self.road.evaluate(t)
            v_cmd = controller.compute_voltage(t, st)
            derivs, _ = self.model.equations_of_motion(t, st, zr, zr_dot, v_cmd)
            return derivs

        for i in range(n_points - 1):
            t_curr = t_eval[i]
            for _ in range(substeps):
                k1 = f_sys(t_curr, curr_state)
                k2 = f_sys(t_curr + 0.5 * h, curr_state + 0.5 * h * k1)
                k3 = f_sys(t_curr + 0.5 * h, curr_state + 0.5 * h * k2)
                k4 = f_sys(t_curr + h, curr_state + h * k3)
                curr_state = curr_state + (h / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
                t_curr += h
            states[:, i + 1] = curr_state

        return self._post_process(t_eval, states, controller)

    def _post_process(
        self,
        time: np.ndarray,
        states: np.ndarray,
        controller: BaseSuspensionController
    ) -> SimulationResult:
        n_points = len(time)
        road_elevation = np.zeros(n_points)
        road_velocity = np.zeros(n_points)
        f_mr = np.zeros(n_points)
        v_cmd = np.zeros(n_points)
        u_effective = states[6]
        z_s_ddot = np.zeros(n_points)
        z_u_ddot = np.zeros(n_points)
        susp_deflection = states[0] - states[2]
        susp_velocity = states[1] - states[3]
        tire_deflection = np.zeros(n_points)
        f_tire = np.zeros(n_points)
        power = np.zeros(n_points)

        for i in range(n_points):
            t_i = time[i]
            st_i = states[:, i]
            zr, zr_dot = self.road.evaluate(t_i)
            v = controller.compute_voltage(t_i, st_i)
            _, aux = self.model.equations_of_motion(t_i, st_i, zr, zr_dot, v)

            road_elevation[i] = zr
            road_velocity[i] = zr_dot
            f_mr[i] = aux["f_mr"]
            v_cmd[i] = v
            z_s_ddot[i] = aux["z_s_ddot"]
            z_u_ddot[i] = aux["z_u_ddot"]
            tire_deflection[i] = aux["tire_deflection"]
            f_tire[i] = aux["f_tire"]
            power[i] = aux["power_dissipated"]

        return SimulationResult(
            time=time,
            states=states,
            road_elevation=road_elevation,
            road_velocity=road_velocity,
            f_mr=f_mr,
            v_cmd=v_cmd,
            u_effective=u_effective,
            z_s_ddot=z_s_ddot,
            z_u_ddot=z_u_ddot,
            susp_deflection=susp_deflection,
            susp_velocity=susp_velocity,
            tire_deflection=tire_deflection,
            f_tire=f_tire,
            power=power,
            controller_name=controller.name,
            color=controller.color
        )
