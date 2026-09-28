"""
Unit Tests for 2-DOF Quarter-Car Dynamics and Mechanics.

Author:
    W. M. Baig
"""

import pytest
import numpy as np
from quarter_car.parameters import QuarterCarParameters
from quarter_car.vehicle import QuarterCarModel


def test_quarter_car_natural_frequencies():
    """Verify natural frequencies match undamped formulas."""
    params = QuarterCarParameters(m_s=320.0, m_u=40.0, k_s=22000.0, k_t=190000.0)
    
    expected_f_s = (1.0 / (2.0 * np.pi)) * np.sqrt(22000.0 / 320.0)
    expected_f_u = (1.0 / (2.0 * np.pi)) * np.sqrt((190000.0 + 22000.0) / 40.0)

    assert np.isclose(params.sprung_natural_frequency_hz, expected_f_s, rtol=1e-4)
    assert np.isclose(params.unsprung_natural_frequency_hz, expected_f_u, rtol=1e-4)
    # Check physical range: body bounce ~1.3 Hz, wheel hop ~11.6 Hz
    assert 1.0 < params.sprung_natural_frequency_hz < 1.6
    assert 10.0 < params.unsprung_natural_frequency_hz < 13.0


def test_quarter_car_dynamic_equilibrium():
    """Verify state derivatives evaluate cleanly without NaN or infinite values."""
    model = QuarterCarModel()
    state = np.array([0.01, 0.05, 0.005, -0.02, 0.002, 0.0001, 1.0])
    
    derivs, aux = model.equations_of_motion(t=0.1, state=state, z_r=0.01, z_r_dot=0.0, v_cmd=1.5)
    
    assert derivs.shape == (7,)
    assert not np.any(np.isnan(derivs))
    assert not np.any(np.isinf(derivs))
    assert "f_mr" in aux
    assert "f_spring" in aux
    assert "f_tire" in aux
