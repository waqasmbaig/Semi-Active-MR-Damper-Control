"""
Quarter-Car Semi-Active Suspension Simulation Package with Spencer Modified Bouc-Wen MR Damper.

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

from quarter_car.parameters import QuarterCarParameters, MRDamperParameters
from quarter_car.mr_damper import SpencerModifiedBoucWenMRDamper
from quarter_car.vehicle import QuarterCarModel
from quarter_car.controllers import (
    BaseSuspensionController,
    PassiveController,
    BasicPassiveDamperController,
    SkyhookController,
    GroundhookController,
    HybridSkyhookGroundhookController,
)
from quarter_car.road_profiles import (
    BaseRoadProfile,
    HaversineBumpRoad,
    ISO8608RandomRoad,
    ChirpHarmonicRoad,
    StepRoad,
)
from quarter_car.metrics import compute_vibration_metrics
from quarter_car.simulator import QuarterCarSimulator, SimulationResult

__all__ = [
    "QuarterCarParameters",
    "MRDamperParameters",
    "SpencerModifiedBoucWenMRDamper",
    "QuarterCarModel",
    "BaseSuspensionController",
    "PassiveController",
    "BasicPassiveDamperController",
    "SkyhookController",
    "GroundhookController",
    "HybridSkyhookGroundhookController",
    "BaseRoadProfile",
    "HaversineBumpRoad",
    "ISO8608RandomRoad",
    "ChirpHarmonicRoad",
    "StepRoad",
    "compute_vibration_metrics",
    "QuarterCarSimulator",
    "SimulationResult",
]

__version__ = "1.0.0"
__author__ = "W. M. Baig"
