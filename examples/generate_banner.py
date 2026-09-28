"""
Generate high-resolution repository banner for Semi-Active-MR-Damper-Control.
"""
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Rectangle, Circle, PathPatch
from matplotlib.path import Path
import matplotlib.patheffects as path_effects

def create_banner():
    # Dimensions: 2400 x 800 at 200 dpi -> 12 x 4 inches
    fig = plt.figure(figsize=(12, 4), dpi=200, facecolor='#0B0F19')
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_facecolor('#0B0F19')
    ax.set_xlim(0, 12)
    ax.set_ylim(0, 4)
    ax.axis('off')

    # 1. Subtle background grid
    for x in np.linspace(0, 12, 49):
        ax.axvline(x, color='#1E293B', lw=0.4, alpha=0.3, zorder=1)
    for y in np.linspace(0, 4, 17):
        ax.axhline(y, color='#1E293B', lw=0.4, alpha=0.3, zorder=1)

    # 2. Glowing background accent gradients (circles / ellipses)
    theta = np.linspace(0, 2*np.pi, 200)
    # Cyan glow on top right
    for r, a in zip(np.linspace(0.5, 3.5, 15), np.linspace(0.12, 0.01, 15)):
        ax.fill(9.8 + r*np.cos(theta), 2.2 + r*0.7*np.sin(theta), color='#06B6D4', alpha=a, lw=0, zorder=2)
    # Indigo/Purple glow on bottom left
    for r, a in zip(np.linspace(0.5, 3.0, 12), np.linspace(0.10, 0.01, 12)):
        ax.fill(2.0 + r*np.cos(theta), 1.0 + r*0.6*np.sin(theta), color='#6366F1', alpha=a, lw=0, zorder=2)

    # 3. Dynamic Waveforms & Hysteresis preview on right side
    # Add waveform callout box
    bbox = FancyBboxPatch((6.8, 0.45), 4.8, 3.1, boxstyle="round,pad=0.1,rounding_size=0.25",
                          facecolor='#111827', edgecolor='#334155', alpha=0.85, lw=1.2, zorder=3)
    ax.add_patch(bbox)

    # Mini title in preview box
    ax.text(7.1, 3.2, "TRANSIENT DYNAMICS COMPARISON", color='#94A3B8', fontsize=7.5, fontweight='bold',
            family='sans-serif', zorder=8)
    
    # Body displacement curves: Basic Passive vs Skyhook
    t_curve = np.linspace(7.1, 11.3, 300)
    tau = t_curve - 7.5
    # Basic Passive decaying oscillation
    y_passive = 1.9 + 0.75 * np.exp(-1.6 * np.maximum(0, tau)) * np.sin(10.0 * np.maximum(0, tau))
    # Skyhook damped response
    y_skyhook = 1.9 + 0.38 * np.exp(-4.2 * np.maximum(0, tau)) * np.sin(9.0 * np.maximum(0, tau))

    # Road bump preview line (ground reference)
    y_road = 0.95 + 0.35 * np.exp(-((t_curve - 7.9) / 0.16)**2)
    ax.plot(t_curve, y_road, color='#64748B', lw=1.4, ls='--', alpha=0.7, zorder=4)
    ax.plot(t_curve, y_passive, color='#EF4444', lw=2.2, alpha=0.85, zorder=5)
    ax.plot(t_curve, y_skyhook, color='#06B6D4', lw=2.6, alpha=0.95, zorder=6)

    # Glowing marker on skyhook peak
    peak_idx = np.argmax(y_skyhook)
    ax.scatter([t_curve[peak_idx]], [y_skyhook[peak_idx]], color='#22D3EE', s=55, zorder=7, edgecolors='white', lw=1.5)

    # Legend items inside box
    ax.plot([7.1, 7.45], [2.85, 2.85], color='#EF4444', lw=2.0, zorder=8)
    ax.text(7.55, 2.79, "Basic Passive (cs = 1500 N·s/m)", color='#FCA5A5', fontsize=7.2, zorder=8)

    ax.plot([7.1, 7.45], [2.55, 2.55], color='#06B6D4', lw=2.2, zorder=8)
    ax.text(7.55, 2.49, "Semi-Active Skyhook (-39.3% Peak Disp)", color='#38BDF8', fontsize=7.2, fontweight='bold', zorder=8)

    ax.plot([7.1, 7.45], [2.25, 2.25], color='#64748B', lw=1.4, ls='--', zorder=8)
    ax.text(7.55, 2.19, "50 mm ISO Obstacle Bump Input", color='#CBD5E1', fontsize=7.2, zorder=8)

    # 4. Left-side Typography & Badges
    # Top Category Pill
    pill = FancyBboxPatch((0.6, 3.25), 3.5, 0.42, boxstyle="round,pad=0.08,rounding_size=0.15",
                          facecolor='#1E293B', edgecolor='#3B82F6', alpha=0.9, lw=1.0, zorder=5)
    ax.add_patch(pill)
    ax.text(0.75, 3.38, "AUTOMOTIVE VEHICLE DYNAMICS & CONTROL", color='#60A5FA', fontsize=7.2,
            fontweight='heavy', family='sans-serif', zorder=6)

    # Main Project Title
    t1 = ax.text(0.6, 2.55, "Semi-Active MR Damper", color='#F8FAFC', fontsize=22,
                 fontweight='heavy', family='sans-serif', zorder=6)
    t1.set_path_effects([path_effects.withStroke(linewidth=3, foreground='#0B0F19')])

    t2 = ax.text(0.6, 2.02, "Quarter-Car Control", color='#38BDF8', fontsize=22,
                 fontweight='heavy', family='sans-serif', zorder=6)
    t2.set_path_effects([path_effects.withStroke(linewidth=3, foreground='#0B0F19')])

    # Subtitle
    ax.text(0.6, 1.62, "Spencer Modified Bouc-Wen Model • Skyhook & Hybrid • Basic Passive Benchmark",
            color='#94A3B8', fontsize=8.2, fontweight='medium', family='sans-serif', zorder=6)
    ax.text(0.6, 1.35, "High-Fidelity Dual-Platform Simulation Suite in MATLAB / Simulink & Pure Python 3",
            color='#64748B', fontsize=7.8, family='sans-serif', zorder=6)

    # Technology Pill Badges at bottom left (2 rows of clean compact badges)
    badges_row1 = [
        ("MATLAB / Simulink", "#E05D44", "#451A03"),
        ("Python 3.9+", "#38BDF8", "#0C4A6E"),
        ("Quarter-Car 2-DOF", "#A855F7", "#3B0764")
    ]
    badges_row2 = [
        ("Stiff Solvers (Radau / ode15s)", "#10B981", "#064E3B"),
        ("IEEE TTE & VTC", "#F59E0B", "#451A03")
    ]

    bx = 0.6
    for label, text_col, bg_col in badges_row1:
        bw = len(label) * 0.088 + 0.32
        p = FancyBboxPatch((bx, 0.85), bw, 0.36, boxstyle="round,pad=0.06,rounding_size=0.10",
                           facecolor=bg_col, edgecolor=text_col, alpha=0.85, lw=0.9, zorder=6)
        ax.add_patch(p)
        ax.text(bx + 0.14, 0.95, label, color=text_col, fontsize=6.8, fontweight='bold',
                family='sans-serif', zorder=7)
        bx += bw + 0.16

    bx = 0.6
    for label, text_col, bg_col in badges_row2:
        bw = len(label) * 0.088 + 0.32
        p = FancyBboxPatch((bx, 0.42), bw, 0.36, boxstyle="round,pad=0.06,rounding_size=0.10",
                           facecolor=bg_col, edgecolor=text_col, alpha=0.85, lw=0.9, zorder=6)
        ax.add_patch(p)
        ax.text(bx + 0.14, 0.52, label, color=text_col, fontsize=6.8, fontweight='bold',
                family='sans-serif', zorder=7)
        bx += bw + 0.16

    # Save high-res PNG
    output_path = 'docs/assets/banner.png'
    plt.savefig(output_path, dpi=200, bbox_inches='tight', pad_inches=0.0)
    plt.close()
    print(f"Banner saved to {output_path}")

if __name__ == '__main__':
    create_banner()
