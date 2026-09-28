<p align="center">
  <img src="docs/assets/banner.png" alt="Semi-Active MR Damper Control Banner" width="100%" style="border-radius: 10px;" />
</p>

<p align="center">
  <h1 align="center">🏎️ Semi-Active-MR-Damper-Control</h1>
  <p align="center"><b>Coupled 2-DOF Quarter-Car Vehicle Handling Dynamics & Spencer Modified Bouc-Wen MR Damper</b></p>
  <p align="center"><i>High-Fidelity Dual-Platform Simulation & Semi-Active Control Framework in MATLAB / Simulink & Pure Python 3</i></p>
</p>

<p align="center">
  <a href="https://opensource.org/licenses/MIT"><img src="https://img.shields.io/badge/License-MIT-blue.svg?style=for-the-badge" alt="License: MIT"/></a>
  <a href="https://www.mathworks.com/products/matlab.html"><img src="https://img.shields.io/badge/MATLAB-R2021b%2B-e05d44.svg?style=for-the-badge&logo=mathworks" alt="MATLAB"/></a>
  <a href="https://www.mathworks.com/products/simulink.html"><img src="https://img.shields.io/badge/Simulink-Quarter_Car_MRD.slx-0076a8.svg?style=for-the-badge&logo=mathworks" alt="Simulink"/></a>
  <a href="https://www.python.org/"><img src="https://img.shields.io/badge/Python-3.9%2B-3776ab.svg?style=for-the-badge&logo=python&logoColor=white" alt="Python"/></a>
  <a href="https://doi.org/10.1109/TTE.2025.3535765"><img src="https://img.shields.io/badge/IEEE%20TTE-10.1109%2FTTE.2025.3535765-orange.svg?style=for-the-badge" alt="IEEE DOI"/></a>
  <a href="https://github.com/waqasmbaig/Semi-Active-MR-Damper-Control"><img src="https://img.shields.io/badge/Topics-Quarter_Car_|_Semi--Active_Control_|_MR_Damper-blueviolet.svg?style=for-the-badge" alt="GitHub Topics"/></a>
</p>

<p align="center">
  <a href="#-key-features"><b>Key Features</b></a> •
  <a href="#-mechanical-architecture"><b>Architecture</b></a> •
  <a href="#-governing-mathematics"><b>Mathematics</b></a> •
  <a href="#-model-parameters"><b>Parameters</b></a> •
  <a href="#-simulink-model-organization"><b>Simulink Model</b></a> •
  <a href="#-python-package-architecture"><b>Python Package</b></a> •
  <a href="#-benchmark-case-studies"><b>Benchmarks</b></a> •
  <a href="#-quick-start"><b>Quick Start</b></a> •
  <a href="#-citation-request"><b>Citations</b></a>
</p>

---

> [!TIP]
> **🔗 MR Damper Companion Ecosystem**:
> - **[Semi-Active-MR-Damper-Control](https://github.com/waqasmbaig/Semi-Active-MR-Damper-Control)**: 2-DOF Quarter-Car vehicle dynamics benchmark, Skyhook / Groundhook / Hybrid semi-active control, and Basic Linear Passive Damper comparison in Python & Simulink.
> - **[MRD-Modified-Bouc-Wen-Model](https://github.com/waqasmbaig/MRD-Modified-Bouc-Wen-Model)**: Core experimental MR damper phenomenological model & dyno hysteresis characterization.
> - **[Modified-Bouc-Wen-Model-Simulation](https://github.com/waqasmbaig/Modified-Bouc-Wen-Model-Simulation)**: Interactive in-browser web application hosted on Google AI Studio.
> - **[MR-Damper-Lab](https://github.com/waqasmbaig/MR-Damper-Lab)**: Digital twin lab tutorials and telemetry analysis toolkit.

> [!NOTE]
> **🏷️ Repository Tags & Topics**: `Quarter car`, `quarter-car`, `quarter-car-model`, `semi-active-control`, `semi-active-mr-damper-control`, `mr-damper`, `magnetorheological-damper`, `skyhook-control`, `groundhook-control`, `bouc-wen-model`, `vehicle-dynamics`, `vibration-control`, `simulink`, `matlab`, `python`.

---

## 🌟 Key Features

<table>
  <tr>
    <td width="50%">
      <h3>⚡ Dual-Platform Framework</h3>
      <p>Seamless execution across <b>MATLAB / Simulink</b> (<code>Quarter_Car_MRD.slx</code>) and standalone <b>Pure Python 3</b> (<code>quarter_car</code> package) with stiff ODE integrators (SciPy Radau and deterministic fixed-step RK4).</p>
    </td>
    <td width="50%">
      <h3>🧲 High-Fidelity Spencer MR Damper</h3>
      <p>Couples the 14-parameter <b>Spencer Modified Bouc-Wen (MBW)</b> phenomenological damper model with nitrogen gas accumulator compliance ($k_1, c_1, x_0$) and continuous electromagnetic coil delay ($\tau \approx 5.26\text{ ms}$).</p>
    </td>
  </tr>
  <tr>
    <td width="50%">
      <h3>🕹️ Advanced Semi-Active Controllers</h3>
      <p>Includes modular implementations of <b>Passive Soft</b> ($0.0\text{ V}$), <b>Passive Hard</b> ($2.0\text{ V}$), <b>Karnopp 2-State Skyhook</b>, <b>Continuous Linear Skyhook</b>, and <b>Hybrid Skyhook-Groundhook</b> control.</p>
    </td>
    <td width="50%">
      <h3>🛣️ Standard ISO Road Excitations</h3>
      <p>Integrated road profiles conforming to international standards: <b>ISO Discrete Obstacle Bump</b> (50 mm Haversine), <b>ISO 8608 Stochastic Road Roughness</b> (Class A&ndash;E), swept-sine harmonic chirps, and step curb strikes.</p>
    </td>
  </tr>
</table>

---

## 📐 Mechanical Architecture

The 2-Degree-of-Freedom quarter-car suspension system is formulated for both conventional passive and controllable semi-active configurations:

<p align="center">
  <img src="docs/assets/Pass_GER_1.jpg" alt="Quarter-Car Model Architectures: (a) Passive vs (b) Semi-active" width="85%" style="border-radius: 8px; box-shadow: 0 4px 16px rgba(0,0,0,0.18);" />
</p>
<p align="center"><b>Figure 1</b>: <i>Schematic diagram of 2-DOF quarter-car suspension system: <b>(a) Passive</b> with standard viscous damper $c_s$, and <b>(b) Semi-active</b> with controllable damping force $F_d$.</i></p>

### Mechanical Parameters & Variables:
- **$m_s$**: Sprung mass representing vehicle chassis quarter body ($320.0\text{ kg}$, body bounce mode $f_{n,s} \approx 1.32\text{ Hz}$).
- **$m_u$**: Unsprung mass representing wheel hub, brake, and tire assembly ($40.0\text{ kg}$, wheel-hop mode $f_{n,u} \approx 11.6\text{ Hz}$).
- **$k_s$**: Suspension primary coil spring stiffness ($22,000.0\text{ N/m}$).
- **$c_s$**: Conventional linear passive damper damping coefficient ($1500.0\text{ N}\cdot\text{s/m}$, damping ratio $\zeta \approx 0.283$) in **(a) Passive**.
- **$F_d$**: Controllable semi-active damping force ($F_d = F_{MR}$) in **(b) Semi-active**.
- **$k_t$**: Tire vertical radial stiffness ($190,000.0\text{ N/m}$).
- **$z_s$**: Sprung mass vertical displacement coordinate ($z$ positive upward, ISO 8855).
- **$z_u$**: Unsprung mass vertical displacement coordinate ($z$ positive upward, ISO 8855).
- **$z_r$**: Road surface profile vertical elevation input.

---

## 🔬 Governing Mathematics

All equations adhere strictly to **SI Base Units** ($\text{m}, \text{s}, \text{kg}, \text{N}, \text{V}, \text{rad}$) and **ISO 8855 coordinate conventions** (vertical axis $z$ positive upward):

### 1. (a) Passive Quarter-Car System Dynamics
In the passive system shown in **Figure 1(a)**, the suspension forces transmitted between the chassis and wheel comprise the linear spring force $k_s (z_s - z_u)$ and the linear viscous damping force $c_s (\dot{z}_s - \dot{z}_u)$.

Applying Newton's second law:

$$
\begin{aligned}
m_s \ddot{z}_s + c_s (\dot{z}_s - \dot{z}_u) + k_s (z_s - z_u) &= 0 \\
m_u \ddot{z}_u - c_s (\dot{z}_s - \dot{z}_u) - k_s (z_s - z_u) + k_t (z_u - z_r) &= 0
\end{aligned}
$$

Expressed explicitly in second-order state form for numerical integration:

$$
\begin{aligned}
\ddot{z}_s &= \frac{-k_s (z_s - z_u) - c_s (\dot{z}_s - \dot{z}_u)}{m_s} \\
\ddot{z}_u &= \frac{k_s (z_s - z_u) + c_s (\dot{z}_s - \dot{z}_u) - k_t (z_u - z_r)}{m_u}
\end{aligned}
$$

### 2. (b) Semi-Active Quarter-Car System Dynamics
In the semi-active system shown in **Figure 1(b)**, the passive damper is replaced by a controllable magnetorheological (MR) damper delivering a variable damping force $F_d$.

Applying Newton's second law:

$$
\begin{aligned}
m_s \ddot{z}_s + k_s (z_s - z_u) + F_d &= 0 \\
m_u \ddot{z}_u - k_s (z_s - z_u) - F_d + k_t (z_u - z_r) &= 0
\end{aligned}
$$

Expressed explicitly in acceleration form:

$$
\begin{aligned}
\ddot{z}_s &= \frac{-k_s (z_s - z_u) - F_d}{m_s} \\
\ddot{z}_u &= \frac{k_s (z_s - z_u) + F_d - k_t (z_u - z_r)}{m_u}
\end{aligned}
$$

Defining relative suspension stroke $x = z_s - z_u$ (extension positive) and dynamic tire deflection $x_t = z_u - z_r$:

$$
\begin{aligned}
\ddot{z}_s &= \frac{-k_s x - F_d}{m_s} \\
\ddot{z}_u &= \frac{k_s x + F_d - k_t x_t}{m_u}
\end{aligned}
$$

### 3. Controllable MR Damper Force Formulation ($F_d$)
Under Spencer's Modified Bouc-Wen phenomenological model, the controllable damping force $F_d = F_{MR}$ is governed by internal accumulator compliance and intermediate displacement $y$:

$$
F_d = c_1 \dot{y} + k_1 (x - x_0)
$$

Equilibrium across internal float plate $y$ yields the intermediate node velocity:

$$
\dot{y} = \frac{1}{c_0 + c_1} \Big[ \alpha z + c_0 \dot{x} + k_0 (x - y) \Big]
$$

The hysteretic restoring variable $z$ evolves according to the Bouc-Wen differential equation:

$$
\dot{z} = A (\dot{x} - \dot{y}) - \beta (\dot{x} - \dot{y}) |z|^n - \gamma |\dot{x} - \dot{y}| |z|^{n-1} z
$$

The coil electromagnetic lag filter governs effective voltage $u$:

$$
\dot{u} = -\eta (u - V_{\text{cmd}})
$$

with field-dependent parameters linearly scaling with effective voltage $u$:

$$
\alpha(u) = \alpha_a + \alpha_b u, \qquad c_0(u) = c_{0a} + c_{0b} u, \qquad c_1(u) = c_{1a} + c_{1b} u
$$

### 4. Semi-Active Skyhook Control Laws

<p align="center">
  <img src="docs/assets/skyhook_switching_surface.png" alt="Skyhook Switching Surface" width="55%" style="border-radius: 8px;" />
</p>

#### 4.1 Classical Karnopp 2-State (On-Off) Skyhook

$$
V_{\text{cmd}} = \begin{cases}
V_{\max} = 2.0\text{ V}, & \text{if } \dot{z}_s (\dot{z}_s - \dot{z}_u) \ge 0 \\
V_{\min} = 0.0\text{ V}, & \text{if } \dot{z}_s (\dot{z}_s - \dot{z}_u) < 0
\end{cases}
$$

#### 4.2 Continuous Linear Skyhook

$$
V_{\text{cmd}} = \begin{cases}
\mathrm{clip}\left(V_{\min} + \dfrac{C_{\text{sky}} |\dot{z}_s|}{F_{\text{ref}}} (V_{\max} - V_{\min}), \, V_{\min}, \, V_{\max}\right), & \text{if } \dot{z}_s (\dot{z}_s - \dot{z}_u) \ge 0 \\
V_{\min}, & \text{otherwise}
\end{cases}
$$

#### 4.3 Hybrid Skyhook-Groundhook

The hybrid switching variable blends sprung body velocity $\dot{z}_s$ and unsprung wheel velocity $\dot{z}_u$ through weighting ratio $\alpha_{\text{hyb}} \in [0, 1]$:

$$
\sigma_{\text{hybrid}} = \alpha_{\text{hyb}} \dot{z}_s - (1 - \alpha_{\text{hyb}}) \dot{z}_u
$$

$$
V_{\text{cmd}} = \begin{cases}
V_{\max}, & \text{if } \sigma_{\text{hybrid}} (\dot{z}_s - \dot{z}_u) \ge 0 \\
V_{\min}, & \text{otherwise}
\end{cases}
$$

---

## 📊 Model Parameters

| Parameter | Symbol | Value | SI Unit | Description |
| :--- | :---: | :---: | :---: | :--- |
| **Sprung Mass** | $m_s$ | `320.0` | $\text{kg}$ | Quarter chassis corner mass |
| **Unsprung Mass** | $m_u$ | `40.0` | $\text{kg}$ | Wheel, tire, and hub mass |
| **Suspension Spring** | $k_s$ | `22000.0` | $\text{N/m}$ | Primary coil spring stiffness |
| **Tire Vertical Stiffness** | $k_t$ | `190000.0` | $\text{N/m}$ | Radial tire vertical stiffness |
| **Basic Passive Damping** | $c_s$ | `1500.0` | $\text{N}\cdot\text{s/m}$ | Linear passive shock absorber ($\zeta \approx 0.28$) |
| **Zero-field damping** | $c_{0a}$ | `784.0` | $\text{N}\cdot\text{s/m}$ | Zero-voltage dashpot damping |
| **Field damping gain** | $c_{0b}$ | `1803.0` | $\text{N}\cdot\text{s/(m}\cdot\text{V)}$ | Damping sensitivity per volt |
| **Dashpot stiffness** | $k_0$ | `3610.0` | $\text{N/m}$ | Post-yield mechanical stiffness |
| **Accumulator damping** | $c_{1a}$ | `14649.0` | $\text{N}\cdot\text{s/m}$ | Nitrogen reservoir damping |
| **Accumulator gain** | $c_{1b}$ | `34622.0` | $\text{N}\cdot\text{s/(m}\cdot\text{V)}$ | Reservoir field sensitivity |
| **Accumulator stiffness** | $k_1$ | `840.0` | $\text{N/m}$ | Gas accumulator compliance |
| **Accumulator offset** | $x_0$ | `0.0245` | $\text{m}$ | Static pre-charge stroke offset ($24.5\text{ mm}$) |
| **Hysteresis scale** | $\alpha_a$ | `12441.0` | $\text{N/m}$ | Base hysteretic yield coefficient |
| **Hysteresis gain** | $\alpha_b$ | `38430.0` | $\text{N/(m}\cdot\text{V)}$ | Voltage hysteretic gain |
| **Shape parameter** | $\gamma$ | `136320.0` | $\text{m}^{-2}$ | Hysteresis loop orientation |
| **Shape parameter** | $\beta$ | `2059020.0` | $\text{m}^{-2}$ | Hysteresis loop width |
| **Restoring multiplier** | $A$ | `58.0` | $-$ | Elastic restoring multiplier |
| **Yield smoothness** | $n$ | `2.0` | $-$ | Transition order exponent |
| **Coil time constant** | $\eta$ | `190.0` | $\text{s}^{-1}$ | Coil rate ($\tau \approx 5.26\text{ ms}$) |

---

## 🎛️ Simulink Model Organization

The model [`matlab/Quarter_Car_MRD.slx`](matlab/Quarter_Car_MRD.slx) is architected into 5 modular, clean subsystems:

```
Quarter_Car_MRD.slx (Top Level)
├── Road_Excitation           [Selectable: Haversine Bump, Chirp, ISO Class C, Step]
│   └── Outputs: z_r, z_r_dot
├── Semi_Active_Controller    [Arbitrates Basic Passive, MR 0V/2V, Skyhook, Continuous, Hybrid]
│   └── Outputs: V_cmd
├── MR_Damper_Bouc_Wen        [Spencer MBW model & Linear Damper: du/dt, alpha, c0, c1, dy/dt, dz/dt]
│   └── Outputs: F_MR, u_eff
├── Quarter_Car_Dynamics      [2-DOF vehicle plant: ms, mu, ks, kt double integrators]
│   └── Outputs: z_s, z_s_dot, z_s_ddot, z_u, z_u_dot, x, x_dot, x_t
└── Performance_Scopes        [Scopes for ride comfort, accelerations, deflection, and force]
```

### Key MATLAB Scripts:
- **[`matlab/init_quarter_car_params.m`](matlab/init_quarter_car_params.m)**: Loads all vehicle, MR damper, basic passive damper ($c_s = 1500\text{ N}\cdot\text{s/m}$), road, controller, and solver settings into the base workspace.
- **[`matlab/run_quarter_car_simulation.m`](matlab/run_quarter_car_simulation.m)**: Automatically executes multi-controller sweeps across Basic Passive, MR Soft/Hard, and Skyhook in Simulink.
- **[`matlab/simulate_quarter_car.m`](matlab/simulate_quarter_car.m)**: Standalone pure MATLAB stiff ODE solver (`ode15s`) that runs with **zero Simulink license requirement**.
- **[`matlab/build_quarter_car_model.m`](matlab/build_quarter_car_model.m)**: Programmatic model builder script that rebuilds `Quarter_Car_MRD.slx` via MATLAB's Simulink API.

---

## 🐍 Python Package Architecture

The Python package is organized as a production-grade, object-oriented library under `quarter_car/`:

```
quarter_car/
├── __init__.py           # Package exports and version
├── parameters.py         # MRDamperParameters and QuarterCarParameters dataclasses
├── mr_damper.py          # SpencerModifiedBoucWenMRDamper class
├── vehicle.py            # QuarterCarModel (coupled 7-state equations of motion)
├── controllers.py        # BasicPassiveDamperController, Passive, Skyhook, Groundhook, Hybrid
├── road_profiles.py      # HaversineBumpRoad, ISO8608RandomRoad, ChirpHarmonicRoad, StepRoad
├── simulator.py          # QuarterCarSimulator (SciPy Radau/BDF and fixed-step RK4)
└── metrics.py            # ISO 2631 ride comfort & ISO 8855 road holding metrics
```

---

## 📈 Benchmark Case Studies

### 1. ISO Discrete Obstacle Bump ($50\text{ mm}$ height, $45\text{ km/h}$)

<p align="center">
  <img src="docs/assets/bump_response_comparison.png" alt="Bump Response Comparison" width="95%" style="border-radius: 8px;" />
</p>

#### Comprehensive Performance Comparison:

| Suspension Configuration | RMS Accel. [$\text{m/s}^2$] | Peak Body Disp. [$\text{mm}$] | Settling Time [$5\%$] | Peak Susp. Travel [$\text{mm}$] | Comparative Performance vs. Basic Passive |
| :--- | :---: | :---: | :---: | :---: | :--- |
| **Basic Passive Damper ($c_s = 1.5\text{ kN}\cdot\text{s/m}$)** | $1.66$ | $14.0$ | $2.01\text{ s}$ | $50.8$ | **Traditional Shock Absorber Baseline** |
| **MR Damper - Passive Soft ($0.0\text{ V}$)** | $1.41$ | $17.0$ | $1.53\text{ s}$ | $58.3$ | Underdamped chassis bounce ($+21.4\%$ peak disp) |
| **MR Damper - Passive Hard ($2.0\text{ V}$)** | $2.51$ | $23.9$ | $2.05\text{ s}$ | $29.4$ | Overdamped harshness ($+70.7\%$ peak disp) |
| **MR Damper - Continuous Skyhook** | **$1.53$** | **$10.5$** | **$1.45\text{ s}$** | $58.3$ | **$25.0\%$ displacement reduction**, $7.8\%$ lower accel |
| **MR Damper - Skyhook (2-State)** | **$2.17$** | **$8.5$** | **$1.36\text{ s}$** | $58.4$ | **$39.3\%$ Displacement Reduction**, **$32.3\%$ faster settling** |

### 2. In-Situ Hysteresis Loops During Vehicle Motion

<p align="center">
  <img src="docs/assets/hysteresis_in_quarter_car.png" alt="In-Situ Hysteresis Loops" width="95%" style="border-radius: 8px;" />
</p>

### 3. ISO 8608 Stochastic Road Roughness (Class C, $50\text{ km/h}$)

<p align="center">
  <img src="docs/assets/random_road_comparison.png" alt="Random Road Comparison" width="95%" style="border-radius: 8px;" />
</p>

---

## 🚀 Quick Start

### 1. Installation
```bash
git clone https://github.com/waqasmbaig/Semi-Active-MR-Damper-Control.git
cd Semi-Active-MR-Damper-Control
pip install -r requirements.txt
```

### 2. Run Python Benchmarks & Tests
```bash
# Execute ISO discrete obstacle bump benchmark
python examples/benchmark_iso_bump.py

# Execute stochastic ISO Class C random road benchmark
python examples/benchmark_random_road.py

# Run comprehensive test suite (13 unit & integration tests)
pytest tests/ -v
```

### 3. Python API Usage Example
```python
from quarter_car import (
    QuarterCarModel,
    HaversineBumpRoad,
    SkyhookController,
    QuarterCarSimulator
)

# 1. Instantiate 2-DOF vehicle plant & road bump
road = HaversineBumpRoad(height=0.05, length=1.0, velocity_kmh=45.0)
plant = QuarterCarModel()
sim = QuarterCarSimulator(vehicle_model=plant, road_profile=road)

# 2. Simulate 2-State Skyhook Semi-Active Controller
controller = SkyhookController(mode="two_state")
res = sim.simulate(controller, t_span=(0.0, 2.5), dt=2e-4, solver="rk4")

# 3. Print quantitative performance indicators
print(res.summary())
print(f"Peak Body Displacement: {res.metrics['peak_disp_mm']:.1f} mm")
print(f"RMS Chassis Accel:      {res.metrics['rms_accel_mps2']:.2f} m/s^2")
```

### 4. MATLAB & Simulink Execution
```matlab
% In MATLAB command window:
cd matlab/
run('init_quarter_car_params.m')          % Load workspace parameters

% Option A: Run organized Simulink model
open_system('Quarter_Car_MRD.slx')
run('run_quarter_car_simulation.m')        % Automated multi-controller sweep

% Option B: Standalone MATLAB simulation (Zero Simulink dependency)
run('simulate_quarter_car.m')
```

---

## 📖 Citation Request

If you use this quarter-car simulation framework, the Spencer Modified Bouc-Wen MR damper model, or code in your academic research, please cite:

```bibtex
@article{yu2025robust,
  author    = {Z. Yu and R. Luo and P. Wu and W. M. Baig and H. Ma and Z. Hou},
  title     = {Robust finite-frequency vibration control of in-wheel motor driving vehicles based on torque coordination and motor suspension},
  journal   = {IEEE Transactions on Transportation Electrification},
  year      = {2025},
  doi       = {10.1109/TTE.2025.3535765}
}

@inproceedings{baig2025adaptive,
  author    = {W. M. Baig and Z. Yu and H. Ma and Z. Hou},
  title     = {Adaptive vibration control of in-wheel motor drive vehicles with preview information},
  booktitle = {Proc. IEEE 101st Vehicular Technology Conference (VTC2025-Spring)},
  address   = {Oslo, Norway},
  year      = {2025},
  doi       = {10.1109/VTC2025-Spring65109.2025.11174543}
}

@inproceedings{baig2017fractional,
  author    = {W. M. Baig and Z. Hou and S. Ijaz},
  title     = {Fractional order controller design for a semi-active suspension system using Nelder-Mead optimization},
  booktitle = {Proc. 29th Chinese Control and Decision Conference (CCDC)},
  pages     = {2808--2813},
  year      = {2017}
}
```

---

## 📄 License
This repository is released under the **[MIT License](LICENSE)**. Copyright &copy; 2025 W. M. Baig.
