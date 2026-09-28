"""
Unit Tests for Spencer Modified Bouc-Wen MR Damper.

Author:
    W. M. Baig
"""

import pytest
import numpy as np
from quarter_car.parameters import MRDamperParameters
from quarter_car.mr_damper import SpencerModifiedBoucWenMRDamper


def test_mr_damper_initialization():
    params = MRDamperParameters()
    damper = SpencerModifiedBoucWenMRDamper(params)
    assert damper.state.shape == (3,)
    assert np.all(damper.state == 0.0)


def test_mr_damper_voltage_scaling():
    """Verify that MR damper force scales up with commanded voltage."""
    damper = SpencerModifiedBoucWenMRDamper()
    state = np.zeros(3)
    x = 0.005      # 5 mm extension
    x_dot = 0.2    # 0.2 m/s extension velocity

    _, f_low = damper.compute_derivatives(0.0, state, x, x_dot, v_cmd=0.0)
    # Give coil time to reach 2V
    state_high_v = np.array([0.0, 0.0, 2.0])
    _, f_high = damper.compute_derivatives(0.0, state_high_v, x, x_dot, v_cmd=2.0)

    assert f_high > f_low, f"High voltage force ({f_high:.1f} N) must exceed low voltage force ({f_low:.1f} N)"


def test_mr_damper_energy_dissipation():
    """Verify positive energy dissipation over a complete sinusoidal stroke cycle."""
    damper = SpencerModifiedBoucWenMRDamper()
    f = 5.0
    omega = 2.0 * np.pi * f
    X0 = 0.010  # 10 mm amplitude

    res = damper.simulate_trajectory(
        t_span=(0.0, 1.0),
        disp_func=lambda t: X0 * np.sin(omega * t),
        vel_func=lambda t: X0 * omega * np.cos(omega * t),
        volt_func=lambda t: 1.0,
        dt=1e-4
    )

    # Work done over the cycle: W = integral(F * dx) = integral(F * v * dt)
    # In steady-state (last 2 cycles):
    idx_ss = res["time"] >= 0.6
    power = res["force"][idx_ss] * res["velocity"][idx_ss]
    net_work = np.trapezoid(power, res["time"][idx_ss])

    assert net_work > 0.0, f"Net energy dissipated ({net_work:.2f} J) must be positive"


def test_coil_inductance_lag():
    """Verify that du/dt = -eta * (u - v_cmd)."""
    damper = SpencerModifiedBoucWenMRDamper()
    state = np.array([0.0, 0.0, 0.0])
    v_cmd = 2.0
    derivs, _ = damper.compute_derivatives(0.0, state, x=0.0, x_dot=0.0, v_cmd=v_cmd)
    
    expected_u_dot = -damper.params.eta * (0.0 - 2.0)
    assert np.isclose(derivs[2], expected_u_dot), f"Coil derivative expected {expected_u_dot}, got {derivs[2]}"
