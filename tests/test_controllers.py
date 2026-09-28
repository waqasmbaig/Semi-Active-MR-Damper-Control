"""
Unit Tests for Semi-Active Suspension Controllers.

Author:
    W. M. Baig
"""

import pytest
import numpy as np
from quarter_car.controllers import (
    PassiveController,
    SkyhookController,
    GroundhookController,
    HybridSkyhookGroundhookController
)


def test_passive_controller():
    c_soft = PassiveController(0.0)
    c_hard = PassiveController(2.0)
    state = np.zeros(7)

    assert c_soft.compute_voltage(0.0, state) == 0.0
    assert c_hard.compute_voltage(0.0, state) == 2.0


def test_skyhook_switching_logic():
    skyhook = SkyhookController(mode="two_state", v_min=0.0, v_max=2.0)

    # Case 1: z_s_dot > 0, v_rel > 0 -> condition > 0 -> V_max
    st1 = np.zeros(7)
    st1[1] = 0.5   # z_s_dot
    st1[3] = 0.1   # z_u_dot => v_rel = 0.4 > 0
    assert skyhook.compute_voltage(0.0, st1) == 2.0

    # Case 2: z_s_dot > 0, v_rel < 0 -> condition < 0 -> V_min
    st2 = np.zeros(7)
    st2[1] = 0.2
    st2[3] = 0.6   # v_rel = -0.4 < 0
    assert skyhook.compute_voltage(0.0, st2) == 0.0

    # Case 3: z_s_dot < 0, v_rel < 0 -> condition > 0 -> V_max
    st3 = np.zeros(7)
    st3[1] = -0.3
    st3[3] = 0.1   # v_rel = -0.4 < 0
    assert skyhook.compute_voltage(0.0, st3) == 2.0


def test_groundhook_switching_logic():
    gh = GroundhookController(v_min=0.0, v_max=2.0)

    # Condition: -z_u_dot * v_rel >= 0
    st1 = np.zeros(7)
    st1[1] = 0.5   # z_s_dot
    st1[3] = -0.2  # z_u_dot -> -z_u_dot = 0.2, v_rel = 0.7 -> condition > 0 -> V_max
    assert gh.compute_voltage(0.0, st1) == 2.0

    st2 = np.zeros(7)
    st2[1] = 0.5
    st2[3] = 0.2   # -z_u_dot = -0.2, v_rel = 0.3 -> condition < 0 -> V_min
    assert gh.compute_voltage(0.0, st2) == 0.0


def test_hybrid_controller():
    hybrid = HybridSkyhookGroundhookController(alpha=0.6, v_min=0.0, v_max=2.0)
    st = np.zeros(7)
    st[1] = 0.5
    st[3] = 0.1
    v = hybrid.compute_voltage(0.0, st)
    assert 0.0 <= v <= 2.0
