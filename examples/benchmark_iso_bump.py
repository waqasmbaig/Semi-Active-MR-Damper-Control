"""
Benchmark Example: 2-DOF Quarter-Car Traversal Over ISO Discrete Obstacle Bump.

Compares:
1. Passive Soft (0.0 V)
2. Passive Hard (2.0 V)
3. Skyhook Semi-Active (2-State Karnopp)
4. Continuous Skyhook
5. Hybrid Skyhook-Groundhook

Author:
    W. M. Baig
"""

import sys
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

repo_root = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(repo_root))

from quarter_car import (
    QuarterCarModel,
    QuarterCarParameters,
    MRDamperParameters,
    HaversineBumpRoad,
    PassiveController,
    SkyhookController,
    HybridSkyhookGroundhookController,
    QuarterCarSimulator
)


def run_bump_benchmark(save_path: str = None):
    # Road bump: 50 mm height, 1 m length, 45 km/h
    road = HaversineBumpRoad(height=0.05, length=1.0, velocity_kmh=45.0, t_start=0.5)
    model = QuarterCarModel()
    simulator = QuarterCarSimulator(vehicle_model=model, road_profile=road)

    controllers = [
        PassiveController(voltage=0.0, name="Passive Soft (0.0 V)", color="#1f77b4"),
        PassiveController(voltage=2.0, name="Passive Hard (2.0 V)", color="#d62728"),
        SkyhookController(mode="two_state", name="Skyhook (2-State)", color="#2ca02c"),
        SkyhookController(mode="continuous", c_sky=2200.0, name="Continuous Skyhook", color="#ff7f0e"),
        HybridSkyhookGroundhookController(alpha=0.65, name="Hybrid Sky-Groundhook", color="#9467bd")
    ]

    t_span = (0.0, 2.5)
    dt = 2e-4
    results = []

    print("\n" + "=" * 90)
    print("QUARTER-CAR SEMI-ACTIVE SUSPENSION BENCHMARK: ISO BUMP EXCITATION")
    print(f"Vehicle Speed: 45 km/h | Bump: 50 mm height x 1.0 m length | Sprung Mass: 320 kg")
    print("=" * 90)

    for ctrl in controllers:
        res = simulator.simulate(ctrl, t_span=t_span, dt=dt, solver="rk4")
        results.append(res)
        print(f"\n{res.summary()}")

    # Comparison summary table
    print("\n" + "-" * 90)
    print(f"{'Controller':<26} | {'RMS Accel [m/s²]':<18} | {'Peak Disp [mm]':<16} | {'Settling [s]':<14} | {'Peak Travel [mm]':<16}")
    print("-" * 90)
    for r in results:
        m = r.metrics
        print(f"{r.controller_name:<26} | {m['rms_accel_mps2']:<18.2f} | {m['peak_disp_mm']:<16.1f} | {m['settling_time_s']:<14.2f} | {m['peak_susp_travel_mm']:<16.1f}")
    print("-" * 90)

    # Plot multi-panel response
    fig, axes = plt.subplots(4, 1, figsize=(11, 10), sharex=True, dpi=300)

    # 1. Road elevation & Sprung mass displacement
    ax1 = axes[0]
    t_arr = results[0].time
    road_elev_mm = results[0].road_elevation * 1000.0
    ax1.plot(t_arr, road_elev_mm, 'k--', label="Road Profile [mm]", alpha=0.5, lw=1.5)
    for r in results:
        ax1.plot(r.time, r.z_s * 1000.0, label=r.controller_name, color=r.color, lw=1.8)
    ax1.set_ylabel("Body Disp. [mm]")
    ax1.set_title("Sprung Mass Vertical Displacement (Ride Comfort)", fontweight="bold")
    ax1.grid(True, alpha=0.4)
    ax1.legend(loc="upper right", frameon=True, fontsize=9)

    # 2. Sprung Mass Acceleration
    ax2 = axes[1]
    for r in results:
        ax2.plot(r.time, r.z_s_ddot, label=r.controller_name, color=r.color, lw=1.8)
    ax2.set_ylabel("Body Accel. [m/s²]")
    ax2.set_title("Sprung Mass Vertical Acceleration (ISO 2631 Comfort)", fontweight="bold")
    ax2.grid(True, alpha=0.4)

    # 3. Suspension Travel
    ax3 = axes[2]
    for r in results:
        ax3.plot(r.time, r.susp_deflection * 1000.0, label=r.controller_name, color=r.color, lw=1.8)
    ax3.set_ylabel("Deflection [mm]")
    ax3.set_title("Relative Suspension Deflection ($z_s - z_u$)", fontweight="bold")
    ax3.grid(True, alpha=0.4)

    # 4. MR Damper Force
    ax4 = axes[3]
    for r in results:
        ax4.plot(r.time, r.f_mr, label=r.controller_name, color=r.color, lw=1.8)
    ax4.set_ylabel("Force [N]")
    ax4.set_xlabel("Time [s]")
    ax4.set_title("MR Damper Output Force ($F_{MR}$)", fontweight="bold")
    ax4.grid(True, alpha=0.4)

    plt.tight_layout()
    if save_path:
        fig.savefig(save_path)
        print(f"\n[Saved figure]: {save_path}")
    plt.close(fig)
    return results


if __name__ == "__main__":
    out_img = repo_root / "docs" / "assets" / "bump_response_comparison.png"
    run_bump_benchmark(str(out_img))
