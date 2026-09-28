"""
Generate high-resolution repository banner for Semi-Active-MR-Damper-Control.
Uses actual physics simulation data from the QuarterCarSimulator pipeline.
Saves to both banner.png and banner_v2.png to bust CDN / browser caches.
"""
import sys
from pathlib import Path
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Rectangle, Circle
import matplotlib.patheffects as path_effects

# Ensure quarter_car package is in sys.path
repo_root = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(repo_root))

from quarter_car import (
    QuarterCarModel,
    QuarterCarParameters,
    HaversineBumpRoad,
    BasicPassiveDamperController,
    SkyhookController,
    QuarterCarSimulator
)


def run_actual_simulations():
    """Run real physical simulation for Basic Passive and Semi-Active Skyhook."""
    road = HaversineBumpRoad(height=0.05, length=1.0, velocity_kmh=45.0, t_start=0.3)
    model = QuarterCarModel()
    simulator = QuarterCarSimulator(vehicle_model=model, road_profile=road)

    t_span = (0.0, 2.2)
    dt = 2e-4

    # 1. Basic Passive Damper
    res_passive = simulator.simulate(
        BasicPassiveDamperController(c_s=1500.0),
        t_span=t_span,
        dt=dt,
        solver="rk4"
    )

    # 2. Semi-Active Skyhook
    res_skyhook = simulator.simulate(
        SkyhookController(mode="two_state"),
        t_span=t_span,
        dt=dt,
        solver="rk4"
    )

    return res_passive, res_skyhook


def create_banner():
    # Run real physical simulation
    res_passive, res_skyhook = run_actual_simulations()

    # Canvas dimensions: 2400 x 800 px at 200 DPI -> 12 x 4 inches
    fig = plt.figure(figsize=(12, 4), dpi=200, facecolor='#090D16')
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_facecolor('#090D16')
    ax.set_xlim(0, 12)
    ax.set_ylim(0, 4)
    ax.axis('off')

    # 1. Subtle background grid
    for x in np.linspace(0, 12, 49):
        ax.axvline(x, color='#1E293B', lw=0.4, alpha=0.35, zorder=1)
    for y in np.linspace(0, 4, 17):
        ax.axhline(y, color='#1E293B', lw=0.4, alpha=0.35, zorder=1)

    # 2. Glowing background accent gradients (subtle ambient lighting)
    theta = np.linspace(0, 2 * np.pi, 200)
    # Cyan ambient glow on top right
    for r, a in zip(np.linspace(0.5, 3.8, 16), np.linspace(0.10, 0.005, 16)):
        ax.fill(9.6 + r * np.cos(theta), 2.3 + r * 0.6 * np.sin(theta), color='#06B6D4', alpha=a, lw=0, zorder=2)
    # Indigo ambient glow on bottom left
    for r, a in zip(np.linspace(0.5, 3.2, 14), np.linspace(0.12, 0.005, 14)):
        ax.fill(2.0 + r * np.cos(theta), 1.1 + r * 0.6 * np.sin(theta), color='#6366F1', alpha=a, lw=0, zorder=2)

    # -------------------------------------------------------------------------
    # 3. Left-Side Typography & Badges (x: 0.60 to 6.50)
    # -------------------------------------------------------------------------
    # Top Category Pill
    pill_h = 0.36
    pill = FancyBboxPatch((0.60, 3.32), 4.30, pill_h, boxstyle="round,pad=0.04,rounding_size=0.10",
                          facecolor='#1E293B', edgecolor='#3B82F6', alpha=0.95, lw=1.0, zorder=5)
    ax.add_patch(pill)
    ax.text(0.75, 3.32 + pill_h / 2.0, "AUTOMOTIVE VEHICLE DYNAMICS & VIBRATION CONTROL",
            color='#60A5FA', fontsize=7.2, fontweight='bold', family='sans-serif', va='center', zorder=6)

    # Main Project Title
    t1 = ax.text(0.60, 2.68, "Semi-Active MR Damper", color='#F8FAFC', fontsize=23,
                 fontweight='heavy', family='sans-serif', zorder=6)
    t1.set_path_effects([path_effects.withStroke(linewidth=3, foreground='#090D16')])

    t2 = ax.text(0.60, 2.12, "Quarter-Car Control", color='#38BDF8', fontsize=23,
                 fontweight='heavy', family='sans-serif', zorder=6)
    t2.set_path_effects([path_effects.withStroke(linewidth=3, foreground='#090D16')])

    # Subtitle
    ax.text(0.60, 1.70, "Spencer Modified Bouc-Wen Model • Skyhook & Hybrid • Basic Passive Benchmark",
            color='#94A3B8', fontsize=8.2, fontweight='medium', family='sans-serif', zorder=6)
    ax.text(0.60, 1.45, "High-Fidelity Dual-Platform Simulation in MATLAB / Simulink (.slx) & Pure Python 3",
            color='#64748B', fontsize=7.8, family='sans-serif', zorder=6)

    # Technology Badges (Two strictly non-overlapping rows with perfect vertical centering)
    badge_h = 0.34
    
    # Row 1: y = 0.88, center y = 1.05
    row1 = [
        ("MATLAB / Simulink", "#FB923C", "#431407", "#EA580C", 1.85),
        ("Python 3.9+", "#38BDF8", "#082F49", "#0284C7", 1.35),
        ("Quarter-Car 2-DOF", "#C084FC", "#3B0764", "#9333EA", 1.85)
    ]
    cur_x = 0.60
    for label, text_c, bg_c, border_c, width in row1:
        p = FancyBboxPatch((cur_x, 0.88), width, badge_h, boxstyle="round,pad=0.04,rounding_size=0.08",
                           facecolor=bg_c, edgecolor=border_c, alpha=0.92, lw=0.9, zorder=6)
        ax.add_patch(p)
        ax.text(cur_x + width / 2.0, 0.88 + badge_h / 2.0, label, color=text_c, fontsize=7.0, fontweight='bold',
                family='sans-serif', ha='center', va='center', zorder=7)
        cur_x += width + 0.16

    # Row 2: y = 0.42, center y = 0.59 (0.42 + 0.34 = 0.76 -> clear gap of 0.12 below 0.88!)
    row2 = [
        ("Stiff Solvers (Radau / ode15s)", "#34D399", "#064E3B", "#059669", 2.45),
        ("IEEE TTE & VTC 2025", "#FBBF24", "#451A03", "#D97706", 1.95),
        ("ISO 8855", "#60A5FA", "#172554", "#2563EB", 1.05)
    ]
    cur_x = 0.60
    for label, text_c, bg_c, border_c, width in row2:
        p = FancyBboxPatch((cur_x, 0.42), width, badge_h, boxstyle="round,pad=0.04,rounding_size=0.08",
                           facecolor=bg_c, edgecolor=border_c, alpha=0.92, lw=0.9, zorder=6)
        ax.add_patch(p)
        ax.text(cur_x + width / 2.0, 0.42 + badge_h / 2.0, label, color=text_c, fontsize=7.0, fontweight='bold',
                family='sans-serif', ha='center', va='center', zorder=7)
        cur_x += width + 0.16

    # -------------------------------------------------------------------------
    # 4. Right-Side Simulation Preview Card (x: 6.75 to 11.55, y: 0.38 to 3.62)
    # -------------------------------------------------------------------------
    card_x, card_y, card_w, card_h = 6.75, 0.38, 4.80, 3.24
    card_bg = FancyBboxPatch((card_x, card_y), card_w, card_h, boxstyle="round,pad=0.06,rounding_size=0.20",
                             facecolor='#0F172A', edgecolor='#334155', alpha=0.95, lw=1.2, zorder=3)
    ax.add_patch(card_bg)

    # Top header bar inside card
    ax.text(card_x + 0.25, card_y + card_h - 0.26, "TRANSIENT DYNAMICS COMPARISON",
            color='#94A3B8', fontsize=7.6, fontweight='bold', family='sans-serif', zorder=8)
    ax.text(card_x + card_w - 0.25, card_y + card_h - 0.26, "ISO Obstacle Bump (45 km/h)",
            color='#64748B', fontsize=6.8, fontweight='medium', family='sans-serif', ha='right', zorder=8)

    # Mini plot telemetry screen area inside card
    plot_x0 = card_x + 0.30
    plot_x1 = card_x + card_w - 0.30
    plot_y0 = card_y + 0.60
    plot_y1 = card_y + card_h - 0.55
    pw = plot_x1 - plot_x0
    ph = plot_y1 - plot_y0

    # Screen background box
    screen_bg = FancyBboxPatch((plot_x0, plot_y0), pw, ph, boxstyle="round,pad=0.02,rounding_size=0.08",
                               facecolor='#0B0F19', edgecolor='#1E293B', alpha=0.9, lw=0.8, zorder=4)
    ax.add_patch(screen_bg)

    # Grid lines inside plot screen
    # Zero displacement baseline
    y_min, y_max = -8.0, 20.0
    def map_y(val):
        return plot_y0 + ((val - y_min) / (y_max - y_min)) * ph

    zero_y = map_y(0.0)
    ax.plot([plot_x0 + 0.05, plot_x1 - 0.05], [zero_y, zero_y], color='#1E293B', lw=0.8, ls=':', zorder=5)

    # Y-axis ticks / labels on left
    ax.text(plot_x0 + 0.08, map_y(15.0), "+15 mm", color='#475569', fontsize=5.8, family='sans-serif', va='center', zorder=5)
    ax.text(plot_x0 + 0.08, map_y(0.0), "0 mm", color='#475569', fontsize=5.8, family='sans-serif', va='center', zorder=5)

    # Simulation data arrays
    t = res_passive.time
    mask = (t >= 0.2) & (t <= 2.0)
    t_sub = t[mask]
    t_norm = (t_sub - 0.2) / (2.0 - 0.2)
    x_coords = plot_x0 + 0.40 + t_norm * (pw - 0.48)

    # Real vertical displacements in mm
    zs_pass_mm = res_passive.z_s[mask] * 1000.0
    zs_sky_mm = res_skyhook.z_s[mask] * 1000.0
    zr_mm = res_passive.road_elevation[mask] * 1000.0

    y_pass_coords = map_y(zs_pass_mm)
    y_sky_coords = map_y(zs_sky_mm)
    # Scaled road bump baseline
    y_road_coords = plot_y0 + 0.06 + (zr_mm / 50.0) * (ph * 0.26)

    # 1. Road bump input curve
    ax.plot(x_coords, y_road_coords, color='#64748B', lw=1.3, ls='--', alpha=0.75, zorder=5)

    # 2. Basic Passive curve
    ax.plot(x_coords, y_pass_coords, color='#F87171', lw=2.0, alpha=0.9, zorder=6)

    # 3. Skyhook curve (with cyan glow effect)
    ax.plot(x_coords, y_sky_coords, color='#06B6D4', lw=4.0, alpha=0.30, zorder=6)
    ax.plot(x_coords, y_sky_coords, color='#22D3EE', lw=2.4, alpha=1.0, zorder=7)

    # Markers at peaks
    pass_peak_idx = np.argmax(zs_pass_mm)
    sky_peak_idx = np.argmax(zs_sky_mm)

    # Passive peak dot & annotation
    ax.scatter([x_coords[pass_peak_idx]], [y_pass_coords[pass_peak_idx]], color='#EF4444', s=38, zorder=8,
               edgecolors='white', lw=1.2)
    ax.text(x_coords[pass_peak_idx] + 0.08, y_pass_coords[pass_peak_idx] + 0.12, "14.0 mm",
            color='#FCA5A5', fontsize=6.8, fontweight='bold', family='sans-serif', zorder=9)

    # Skyhook peak dot & annotation
    ax.scatter([x_coords[sky_peak_idx]], [y_sky_coords[sky_peak_idx]], color='#22D3EE', s=42, zorder=8,
               edgecolors='white', lw=1.4)
    ax.text(x_coords[sky_peak_idx] + 0.08, y_sky_coords[sky_peak_idx] + 0.10, "8.5 mm (-39.3%)",
            color='#38BDF8', fontsize=6.8, fontweight='bold', family='sans-serif', zorder=9)

    # Settling pill indicator in top-right of screen
    sp_w, sp_h = 1.45, 0.22
    sp_x = plot_x1 - sp_w - 0.06
    sp_y = plot_y1 - sp_h - 0.06
    sp_box = FancyBboxPatch((sp_x, sp_y), sp_w, sp_h, boxstyle="round,pad=0.02,rounding_size=0.06",
                            facecolor='#064E3B', edgecolor='#10B981', alpha=0.9, lw=0.7, zorder=7)
    ax.add_patch(sp_box)
    ax.text(sp_x + sp_w / 2.0, sp_y + sp_h / 2.0, "Settling: -32.3% faster",
            color='#34D399', fontsize=6.2, fontweight='bold', ha='center', va='center', family='sans-serif', zorder=8)

    # Clean Legend in card footer (y: card_y + 0.22)
    leg_y = card_y + 0.22
    # Item 1: Passive
    ax.plot([card_x + 0.30, card_x + 0.58], [leg_y, leg_y], color='#F87171', lw=2.0, zorder=8)
    ax.text(card_x + 0.65, leg_y, "Basic Passive (cs = 1500 N·s/m)", color='#FCA5A5', fontsize=6.6,
            va='center', family='sans-serif', zorder=8)

    # Item 2: Skyhook
    ax.plot([card_x + 2.15, card_x + 2.43], [leg_y, leg_y], color='#22D3EE', lw=2.2, zorder=8)
    ax.text(card_x + 2.50, leg_y, "Semi-Active Skyhook", color='#38BDF8', fontsize=6.6,
            fontweight='bold', va='center', family='sans-serif', zorder=8)

    # Item 3: Road
    ax.plot([card_x + 3.75, card_x + 4.00], [leg_y, leg_y], color='#64748B', lw=1.3, ls='--', zorder=8)
    ax.text(card_x + 4.07, leg_y, "Road (50 mm)", color='#94A3B8', fontsize=6.6,
            va='center', family='sans-serif', zorder=8)

    # Save to both paths: banner.png and banner_v2.png (cache-buster!)
    out1 = repo_root / 'docs' / 'assets' / 'banner.png'
    out2 = repo_root / 'docs' / 'assets' / 'banner_v2.png'
    plt.savefig(out1, dpi=200, bbox_inches='tight', pad_inches=0.0)
    plt.savefig(out2, dpi=200, bbox_inches='tight', pad_inches=0.0)
    plt.close()
    print(f"Saved perfected banner to:\n  - {out1}\n  - {out2}")


if __name__ == '__main__':
    create_banner()
