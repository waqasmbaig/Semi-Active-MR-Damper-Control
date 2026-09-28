"""
End-to-end integration tests for Quarter-Car Simulation Pipeline.

Author:
    W. M. Baig
"""

import pytest
import numpy as np
from quarter_car.vehicle import QuarterCarModel
from quarter_car.road_profiles import HaversineBumpRoad, ISO8608RandomRoad
from quarter_car.controllers import PassiveController, SkyhookController
from quarter_car.simulator import QuarterCarSimulator


def test_simulation_bump_execution_radau():
    """Verify that Radau stiff solver executes simulation and produces valid signals."""
    simulator = QuarterCarSimulator(
        vehicle_model=QuarterCarModel(),
        road_profile=HaversineBumpRoad(height=0.05, length=1.0, velocity_kmh=45.0)
    )

    controller = SkyhookController(mode="two_state")
    res = simulator.simulate(controller, t_span=(0.0, 1.5), dt=1e-3, solver="radau")

    assert len(res.time) > 100
    assert not np.any(np.isnan(res.z_s))
    assert not np.any(np.isnan(res.f_mr))
    assert res.metrics["peak_disp_mm"] > 0.0
    assert res.metrics["rms_accel_mps2"] > 0.0


def test_simulation_bump_execution_rk4():
    """Verify that RK4 fixed-step solver executes cleanly and matches physical bounds."""
    simulator = QuarterCarSimulator(
        vehicle_model=QuarterCarModel(),
        road_profile=HaversineBumpRoad(height=0.05, length=1.0, velocity_kmh=45.0)
    )

    controller = PassiveController(0.0)
    res = simulator.simulate(controller, t_span=(0.0, 1.5), dt=2e-4, solver="rk4")

    assert len(res.time) > 500
    assert not np.any(np.isnan(res.z_s))
    # Body displacement over 50 mm bump should remain bounded (< 50 mm)
    assert res.metrics["peak_disp_mm"] < 50.0


def test_skyhook_superiority_over_passive_hard():
    """Verify that Skyhook semi-active control yields lower peak body displacement than Passive Hard."""
    road = HaversineBumpRoad(height=0.05, length=1.0, velocity_kmh=45.0)
    sim = QuarterCarSimulator(QuarterCarModel(), road)

    res_hard = sim.simulate(PassiveController(2.0), t_span=(0.0, 2.0), dt=5e-4, solver="radau")
    res_sky = sim.simulate(SkyhookController(mode="two_state"), t_span=(0.0, 2.0), dt=5e-4, solver="radau")

    # Skyhook should achieve lower peak body displacement than stiff passive hard
    assert res_sky.metrics["peak_disp_mm"] < res_hard.metrics["peak_disp_mm"]
