"""
Vibration and Ride Quality Evaluation Metrics (ISO 2631 / ISO 8855 Standards).

Author:
    W. M. Baig
"""

from typing import Dict, Any, Optional
import numpy as np


def compute_vibration_metrics(
    time: np.ndarray,
    z_s: np.ndarray,
    z_s_ddot: np.ndarray,
    susp_deflection: np.ndarray,
    tire_deflection: np.ndarray,
    f_mr: np.ndarray,
    power: Optional[np.ndarray] = None,
    tolerance_pct: float = 5.0
) -> Dict[str, float]:
    """
    Compute standard vehicle dynamics performance metrics for ride comfort,
    suspension travel limits, and road holding.

    Args:
        time: Time array [s]
        z_s: Sprung mass displacement [m]
        z_s_ddot: Sprung mass acceleration [m/s^2]
        susp_deflection: Relative suspension deflection (z_s - z_u) [m]
        tire_deflection: Tire deflection (z_u - z_r) [m]
        f_mr: MR Damper force [N]
        power: Instantaneous power dissipation [W]
        tolerance_pct: Percentage band for settling time calculation (e.g. 5.0%)

    Returns:
        metrics: Dictionary of quantitative summary indicators
    """
    dt = np.mean(np.diff(time))
    total_time = time[-1] - time[0]

    # 1. Ride Comfort (ISO 2631 weighted or raw vertical acceleration)
    rms_accel = float(np.sqrt(np.mean(z_s_ddot ** 2)))
    peak_accel = float(np.max(np.abs(z_s_ddot)))
    crest_factor = float(peak_accel / (rms_accel + 1e-12))

    # 2. Body Displacement & Settling Time
    peak_disp = float(np.max(np.abs(z_s)))
    rms_disp = float(np.sqrt(np.mean(z_s ** 2)))

    # Settling time calculation (time to settle within tolerance of final displacement)
    final_val = z_s[-1]
    peak_deviation = np.max(np.abs(z_s - final_val))
    band = (tolerance_pct / 100.0) * (peak_deviation if peak_deviation > 1e-6 else 1.0)
    
    # Search backwards from end for last point outside band
    outside_indices = np.where(np.abs(z_s - final_val) > band)[0]
    if len(outside_indices) == 0:
        settling_time = 0.0
    else:
        last_idx = outside_indices[-1]
        settling_time = float(time[min(last_idx + 1, len(time) - 1)])

    # 3. Suspension Deflection (Working Space)
    peak_susp_travel = float(np.max(np.abs(susp_deflection)))
    rms_susp_travel = float(np.sqrt(np.mean(susp_deflection ** 2)))

    # 4. Road Holding (Tire Deflection)
    peak_tire_deflection = float(np.max(np.abs(tire_deflection)))
    rms_tire_deflection = float(np.sqrt(np.mean(tire_deflection ** 2)))

    # 5. Damper Force & Energy Dissipation
    peak_mr_force = float(np.max(np.abs(f_mr)))
    rms_mr_force = float(np.sqrt(np.mean(f_mr ** 2)))

    metrics = {
        "rms_accel_mps2": rms_accel,
        "peak_accel_mps2": peak_accel,
        "crest_factor": crest_factor,
        "peak_disp_mm": peak_disp * 1000.0,
        "rms_disp_mm": rms_disp * 1000.0,
        "settling_time_s": settling_time,
        "peak_susp_travel_mm": peak_susp_travel * 1000.0,
        "rms_susp_travel_mm": rms_susp_travel * 1000.0,
        "peak_tire_deflection_mm": peak_tire_deflection * 1000.0,
        "rms_tire_deflection_mm": rms_tire_deflection * 1000.0,
        "peak_mr_force_N": peak_mr_force,
        "rms_mr_force_N": rms_mr_force
    }

    if power is not None:
        metrics["avg_power_dissipated_W"] = float(np.mean(power))

    return metrics
