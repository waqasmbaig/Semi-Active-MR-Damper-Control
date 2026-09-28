"""
Automated Visualization Asset Generator for Quarter-Car MR Damper Simulation.

Generates:
1. bump_response_comparison.png - Time histories across control laws
2. random_road_comparison.png - Stochastic ISO Class C response
3. hysteresis_in_quarter_car.png - In-situ dynamic hysteresis loops (F-D and F-V)
4. skyhook_switching_surface.png - Skyhook phase-plane switching logic
5. quarter_car_schematic.png - Publication-grade mechanical schematic diagram

Author:
    W. M. Baig
"""

import sys
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

repo_root = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(repo_root))

from quarter_car import (
    QuarterCarModel,
    HaversineBumpRoad,
    ISO8608RandomRoad,
    PassiveController,
    SkyhookController,
    HybridSkyhookGroundhookController,
    QuarterCarSimulator
)
from examples.benchmark_iso_bump import run_bump_benchmark
from examples.benchmark_random_road import run_random_road_benchmark


def generate_in_situ_hysteresis(save_path: str):
    """Plot the dynamic force-displacement and force-velocity loops during bump traversal."""
    road = HaversineBumpRoad(height=0.05, length=1.0, velocity_kmh=45.0, t_start=0.5)
    sim = QuarterCarSimulator(QuarterCarModel(), road)

    res_soft = sim.simulate(PassiveController(0.0), t_span=(0.0, 2.0), dt=2e-4)
    res_hard = sim.simulate(PassiveController(2.0), t_span=(0.0, 2.0), dt=2e-4)
    res_sky = sim.simulate(SkyhookController("two_state"), t_span=(0.0, 2.0), dt=2e-4)

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5.5), dpi=300)

    # 1. Force vs Suspension Deflection
    ax1.plot(res_soft.susp_deflection * 1000.0, res_soft.f_mr, label="Passive Soft (0 V)", color="#1f77b4", lw=1.6, alpha=0.85)
    ax1.plot(res_hard.susp_deflection * 1000.0, res_hard.f_mr, label="Passive Hard (2 V)", color="#d62728", lw=1.6, alpha=0.85)
    ax1.plot(res_sky.susp_deflection * 1000.0, res_sky.f_mr, label="Skyhook Semi-Active", color="#2ca02c", lw=1.8)
    ax1.set_xlabel("Suspension Deflection ($z_s - z_u$) [mm]", fontweight="bold")
    ax1.set_ylabel("MR Damper Force [N]", fontweight="bold")
    ax1.set_title("In-Situ Force vs. Displacement (Bump Excursion)", fontweight="bold")
    ax1.grid(True, alpha=0.4)
    ax1.legend(loc="lower right", frameon=True)

    # 2. Force vs Suspension Velocity
    ax2.plot(res_soft.susp_velocity, res_soft.f_mr, label="Passive Soft (0 V)", color="#1f77b4", lw=1.6, alpha=0.85)
    ax2.plot(res_hard.susp_velocity, res_hard.f_mr, label="Passive Hard (2 V)", color="#d62728", lw=1.6, alpha=0.85)
    ax2.plot(res_sky.susp_velocity, res_sky.f_mr, label="Skyhook Semi-Active", color="#2ca02c", lw=1.8)
    ax2.set_xlabel("Piston Relative Velocity ($\dot{z}_s - \dot{z}_u$) [m/s]", fontweight="bold")
    ax2.set_ylabel("MR Damper Force [N]", fontweight="bold")
    ax2.set_title("In-Situ Force vs. Velocity Hysteresis", fontweight="bold")
    ax2.grid(True, alpha=0.4)
    ax2.legend(loc="upper left", frameon=True)

    plt.tight_layout()
    fig.savefig(save_path)
    plt.close(fig)
    print(f"[Generated]: {save_path}")


def generate_skyhook_phase_plane(save_path: str):
    """Plot the 2D Skyhook switching plane with quadrants."""
    v_s_dot = np.linspace(-0.6, 0.6, 300)
    v_rel = np.linspace(-0.8, 0.8, 300)
    V_S, V_REL = np.meshgrid(v_s_dot, v_rel)

    # Skyhook switching criterion: z_s_dot * v_rel >= 0
    Z_switch = np.where(V_S * V_REL >= 0, 2.0, 0.0)

    fig, ax = plt.subplots(figsize=(7, 6), dpi=300)
    c = ax.contourf(V_S, V_REL, Z_switch, levels=[-0.5, 0.5, 2.5], colors=['#e8f4f8', '#d1e7dd'])
    ax.axhline(0, color='black', lw=1.5)
    ax.axvline(0, color='black', lw=1.5)

    ax.text(0.25, 0.35, "V = 2.0 V (ON)\nActive Dissipation", fontsize=11, fontweight='bold', color='#155724', ha='center')
    ax.text(-0.25, -0.35, "V = 2.0 V (ON)\nActive Dissipation", fontsize=11, fontweight='bold', color='#155724', ha='center')
    ax.text(0.25, -0.35, "V = 0.0 V (OFF)\nSoft Damping", fontsize=11, fontweight='bold', color='#0c5460', ha='center')
    ax.text(-0.25, 0.35, "V = 0.0 V (OFF)\nSoft Damping", fontsize=11, fontweight='bold', color='#0c5460', ha='center')

    ax.set_xlabel("Chassis Vertical Velocity $\dot{z}_s$ [m/s]", fontweight="bold", fontsize=11)
    ax.set_ylabel("Suspension Velocity $\dot{x} = \dot{z}_s - \dot{z}_u$ [m/s]", fontweight="bold", fontsize=11)
    ax.set_title("Karnopp Skyhook Semi-Active Switching Surface", fontweight="bold", fontsize=12)
    ax.grid(True, alpha=0.3, ls='--')

    plt.tight_layout()
    fig.savefig(save_path)
    plt.close(fig)
    print(f"[Generated]: {save_path}")


def generate_schematic_diagram(save_path: str):
    """Draw a vector mechanical schematic of the 2-DOF Quarter-Car with MR Damper."""
    fig, ax = plt.subplots(figsize=(8, 9), dpi=300)
    ax.set_xlim(-2.5, 2.5)
    ax.set_ylim(-0.5, 5.5)
    ax.axis('off')

    # 1. Sprung Mass (Car Body Quarter)
    body_box = patches.FancyBboxPatch((-1.6, 3.8), 3.2, 1.2, boxstyle="round,pad=0.1", ec="#1b365d", fc="#e3f2fd", lw=2.5)
    ax.add_patch(body_box)
    ax.text(0, 4.4, "Sprung Mass ($m_s$)\nChassis Body Corner (320 kg)", ha='center', va='center', fontsize=11, fontweight='bold', color="#0d47a1")

    # Arrow for z_s
    ax.annotate("", xy=(1.9, 4.8), xytext=(1.9, 3.9), arrowprops=dict(arrowstyle="->", lw=2, color="#0d47a1"))
    ax.text(2.05, 4.35, "$z_s(t)$\n$\dot{z}_s(t)$", fontsize=11, color="#0d47a1", va='center')

    # 2. Suspension components (Spring k_s and MR Damper F_MR)
    # Left: Coil Spring k_s
    # Draw zigzag spring
    spring_x = -0.8
    y_spring = np.linspace(2.2, 3.8, 40)
    x_spring = spring_x + 0.15 * np.sin(np.linspace(0, 8 * np.pi, 40))
    ax.plot(x_spring, y_spring, color="#c62828", lw=2.5)
    ax.text(spring_x - 0.5, 3.0, "Spring\n$k_s = 22\\text{ kN/m}$", ha='center', fontsize=10, color="#b71c1c", fontweight='bold')

    # Right: MR Damper (Cylinder + Piston)
    damper_x = 0.8
    # Outer cylinder
    cyl = patches.Rectangle((damper_x - 0.25, 2.2), 0.5, 0.9, ec="#2e7d32", fc="#e8f5e9", lw=2.0)
    ax.add_patch(cyl)
    # Piston rod & plate
    ax.plot([damper_x, damper_x], [3.8, 2.7], color="#1b5e20", lw=3.0)
    ax.plot([damper_x - 0.2, damper_x + 0.2], [2.7, 2.7], color="#1b5e20", lw=3.5)
    # Damper label
    ax.text(damper_x + 0.65, 3.0, "MR Damper\nSpencer MBW\n$F_{MR}(x, \dot{x}, V)$", ha='center', fontsize=10, color="#1b5e20", fontweight='bold')

    # Relative suspension deflection annotation
    ax.annotate("", xy=(-0.05, 3.75), xytext=(-0.05, 2.25), arrowprops=dict(arrowstyle="<->", lw=1.8, color="#424242"))
    ax.text(0.05, 3.0, "$x = z_s - z_u$", fontsize=10, color="#424242", va='center')

    # 3. Unsprung Mass (Wheel / Hub / Brake)
    wheel_box = patches.FancyBboxPatch((-1.2, 1.6), 2.4, 0.6, boxstyle="round,pad=0.08", ec="#e65100", fc="#fff3e0", lw=2.2)
    ax.add_patch(wheel_box)
    ax.text(0, 1.9, "Unsprung Mass ($m_u$)\nWheel / Hub Assembly (40 kg)", ha='center', va='center', fontsize=10, fontweight='bold', color="#bf360c")

    # Arrow for z_u
    ax.annotate("", xy=(1.5, 2.2), xytext=(1.5, 1.6), arrowprops=dict(arrowstyle="->", lw=2, color="#e65100"))
    ax.text(1.65, 1.9, "$z_u(t)$", fontsize=11, color="#e65100", va='center')

    # 4. Tire compliance (k_t)
    tire_x = 0.0
    y_tire = np.linspace(0.5, 1.6, 25)
    x_tire = tire_x + 0.15 * np.sin(np.linspace(0, 5 * np.pi, 25))
    ax.plot(x_tire, y_tire, color="#37474f", lw=2.5)
    ax.text(0.65, 1.05, "Tire Radial\n$k_t = 190\\text{ kN/m}$", fontsize=10, color="#263238", fontweight='bold')

    # 5. Road Surface Excitation (z_r)
    road_y = 0.5
    ax.plot([-2.0, 2.0], [road_y, road_y], color="#212121", lw=3.0)
    # Hatching ground
    for hx in np.linspace(-1.9, 1.9, 20):
        ax.plot([hx, hx - 0.15], [road_y, road_y - 0.2], color="#757575", lw=1.5)
    ax.text(0, 0.15, "Road Surface Profile $z_r(t)$ [ISO Obstacle Bump / Random Roughness]", ha='center', fontsize=10, fontweight='bold')

    # Arrow for z_r
    ax.annotate("", xy=(1.7, 0.9), xytext=(1.7, 0.5), arrowprops=dict(arrowstyle="->", lw=2, color="#212121"))
    ax.text(1.85, 0.7, "$z_r(t)$", fontsize=11, color="#212121", va='center')

    plt.tight_layout()
    fig.savefig(save_path)
    plt.close(fig)
    print(f"[Generated]: {save_path}")


def main():
    assets_dir = repo_root / "docs" / "assets"
    assets_dir.mkdir(parents=True, exist_ok=True)

    print("\n--- Generating Quarter-Car Simulation Visualizations ---")
    run_bump_benchmark(str(assets_dir / "bump_response_comparison.png"))
    run_random_road_benchmark(str(assets_dir / "random_road_comparison.png"))
    generate_in_situ_hysteresis(str(assets_dir / "hysteresis_in_quarter_car.png"))
    generate_skyhook_phase_plane(str(assets_dir / "skyhook_switching_surface.png"))
    generate_schematic_diagram(str(assets_dir / "quarter_car_schematic.png"))
    print("\n--- All visualizations generated successfully! ---")


if __name__ == "__main__":
    main()
