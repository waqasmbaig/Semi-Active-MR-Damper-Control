"""
Benchmark Example: 2-DOF Quarter-Car Traversal Over ISO 8608 Random Road Roughness.

Compares:
1. Passive Soft (0.0 V)
2. Passive Hard (2.0 V)
3. Skyhook Semi-Active (2-State)
4. Hybrid Skyhook-Groundhook

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
    ISO8608RandomRoad,
    BasicPassiveDamperController,
    PassiveController,
    SkyhookController,
    HybridSkyhookGroundhookController,
    QuarterCarSimulator
)


def run_random_road_benchmark(save_path: str = None):
    # ISO Class C (Average surface) at 50 km/h for 4.0 seconds
    road = ISO8608RandomRoad(road_class="C", velocity_kmh=50.0, duration=4.0, dt=1e-4, seed=101)
    model = QuarterCarModel()
    simulator = QuarterCarSimulator(vehicle_model=model, road_profile=road)

    controllers = [
        BasicPassiveDamperController(c_s=1500.0, name="Basic Passive (Linear)", color="#7f7f7f"),
        PassiveController(voltage=0.0, name="MR Passive Soft (0.0 V)", color="#1f77b4"),
        PassiveController(voltage=2.0, name="MR Passive Hard (2.0 V)", color="#d62728"),
        SkyhookController(mode="two_state", name="Skyhook (2-State)", color="#2ca02c"),
        HybridSkyhookGroundhookController(alpha=0.60, name="Hybrid Sky-Groundhook", color="#9467bd")
    ]

    t_span = (0.0, 4.0)
    dt = 5e-4
    results = []

    print("\n" + "=" * 90)
    print("QUARTER-CAR BENCHMARK: ISO 8608 CLASS C RANDOM ROAD ROUGHNESS")
    print(f"Vehicle Speed: 50 km/h | Road Class: C (Average) | Duration: 4.0 s")
    print("=" * 90)

    for ctrl in controllers:
        res = simulator.simulate(ctrl, t_span=t_span, dt=dt, solver="rk4")
        results.append(res)
        print(f"\n{res.summary()}")

    fig, axes = plt.subplots(3, 1, figsize=(11, 8.5), sharex=True, dpi=300)

    # 1. Road elevation & Sprung mass displacement
    ax1 = axes[0]
    ax1.plot(results[0].time, results[0].road_elevation * 1000.0, 'k--', alpha=0.35, label="Road Profile [mm]")
    for r in results:
        ax1.plot(r.time, r.z_s * 1000.0, label=r.controller_name, color=r.color, lw=1.6)
    ax1.set_ylabel("Body Disp. [mm]")
    ax1.set_title("Sprung Mass Vertical Displacement (ISO Class C Road)", fontweight="bold")
    ax1.grid(True, alpha=0.4)
    ax1.legend(loc="upper right", frameon=True, fontsize=9)

    # 2. Sprung mass acceleration
    ax2 = axes[1]
    for r in results:
        ax2.plot(r.time, r.z_s_ddot, label=r.controller_name, color=r.color, lw=1.5)
    ax2.set_ylabel("Body Accel. [m/s²]")
    ax2.set_title("Chassis Vertical Acceleration", fontweight="bold")
    ax2.grid(True, alpha=0.4)

    # 3. Dynamic Tire Deflection
    ax3 = axes[2]
    for r in results:
        ax3.plot(r.time, r.tire_deflection * 1000.0, label=r.controller_name, color=r.color, lw=1.5)
    ax3.set_ylabel("Tire Defl. [mm]")
    ax3.set_xlabel("Time [s]")
    ax3.set_title("Dynamic Tire Deflection ($z_u - z_r$) [Road Holding Safety]", fontweight="bold")
    ax3.grid(True, alpha=0.4)

    plt.tight_layout()
    if save_path:
        fig.savefig(save_path)
        print(f"\n[Saved figure]: {save_path}")
    plt.close(fig)
    return results


if __name__ == "__main__":
    out_img = repo_root / "docs" / "assets" / "random_road_comparison.png"
    run_random_road_benchmark(str(out_img))
