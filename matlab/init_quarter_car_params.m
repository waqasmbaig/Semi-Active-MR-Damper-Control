%% Parameter Initialization for Quarter-Car Model with Spencer Modified Bouc-Wen MR Damper
% Author: W. M. Baig
% Model file: Quarter_Car_MRD.slx
%
% Citation Request:
%   [1] Z. Yu, R. Luo, P. Wu, W. M. Baig, H. Ma, Z. Hou, "Robust finite-frequency vibration
%       control of in-wheel motor driving vehicles based on torque coordination and motor
%       suspension," IEEE Transactions on Transportation Electrification, 2025.
%       doi: 10.1109/TTE.2025.3535765
%   [2] W. M. Baig, Z. Yu, H. Ma, Z. Hou, "Adaptive vibration control of in-wheel motor
%       drive vehicles with preview information," in Proc. IEEE 101st Vehicular Technology
%       Conference (VTC2025-Spring), Oslo, Norway, 2025.
%       doi: 10.1109/VTC2025-Spring65109.2025.11174543
%   [3] W. M. Baig, Z. Hou, S. Ijaz, "Fractional order controller design for a semi-active
%       suspension system using Nelder-Mead optimization," CCDC, 2017, pp. 2808-2813.

clear;
clc;

%% 1. Simulation Runtime Settings
SimTime  = 2.5;                % Simulation duration [s]
dt_fixed = 5e-5;               % Fixed-step integration size [s] (20 kHz for Bouc-Wen stability)
g        = 9.80665;            % Gravitational acceleration [m/s^2]

%% 2. Quarter-Car Vehicle Physical Parameters (ISO SI Units)
m_s      = 320.0;              % Sprung mass (chassis body quarter) [kg]
m_u      = 40.0;               % Unsprung mass (wheel/hub assembly) [kg]
k_s      = 22000.0;            % Suspension coil spring stiffness [N/m]
c_s_pass = 0.0;                % Auxiliary parallel passive damping [N*s/m]
c_s_linear = 1500.0;           % Standard basic linear passive shock absorber damping [N*s/m]
k_t      = 190000.0;           % Tire radial vertical stiffness [N/m]
c_t      = 0.0;                % Tire vertical damping [N*s/m]

% Damping ratios and critical values
c_crit   = 2 * sqrt(m_s * k_s); % Critical damping coefficient (~5306.6 N*s/m)
zeta_lin = c_s_linear / c_crit; % Damping ratio of basic passive damper (~0.283)

% Static equilibrium offsets
delta_s0 = (m_s * g) / k_s;    % Static suspension deflection [m]
delta_t0 = ((m_s + m_u) * g) / k_t; % Static tire deflection [m]

% Natural frequencies
fn_sprung   = (1 / (2 * pi)) * sqrt(k_s / m_s);             % Body bounce mode (~1.32 Hz)
fn_unsprung = (1 / (2 * pi)) * sqrt((k_t + k_s) / m_u);     % Wheel hop mode (~11.6 Hz)

%% 3. Spencer Modified Bouc-Wen MR Damper Parameters
% Dashpot & Viscous parameters
c0_a     = 784.0;              % Base dashpot viscous damping [N*s/m]
c0_b     = 1803.0;             % Field-dependent damping sensitivity [N*s/(m*V)]
k0       = 3610.0;             % Post-yield stiffness [N/m]

% Nitrogen gas accumulator compliance
c1_a     = 14649.0;            % Accumulator damping coefficient [N*s/m]
c1_b     = 34622.0;            % Field accumulator damping gain [N*s/(m*V)]
k1       = 840.0;              % Gas accumulator stiffness [N/m]
x0       = 0.0245;             % Accumulator initial displacement offset [m] (24.5 mm)

% Bouc-Wen hysteresis parameters
alpha_a  = 12441.0;            % Base hysteretic restoring force coefficient [N/m]
alpha_b  = 38430.0;            % Field-dependent hysteretic force gain [N/(m*V)]
gamma    = 136320.0;           % Hysteresis loop shape factor [m^-2]
beta     = 2059020.0;          % Hysteresis loop shape factor [m^-2]
A        = 58.0;               % Restoring scale parameter [-]
n        = 2.0;                % Smoothness transition exponent [-]

% Coil electromagnetic dynamics
eta      = 190.0;              % Coil first-order rate constant [s^-1] (tau = 1/eta ~ 5.26 ms)
V_min    = 0.0;                % Minimum rated coil voltage [V]
V_max    = 2.0;                % Maximum rated coil voltage [V]

%% 4. Road Excitation Settings
% RoadType:
%   1 -> ISO Discrete Obstacle Bump (Haversine profile)
%   2 -> Swept-Sine Chirp Excitation (0.5 to 15 Hz)
%   3 -> ISO 8608 Class C Stochastic Roughness (White noise filtered)
%   4 -> Smoothed Step (Curb strike)
RoadType = 1;

% Bump specifications
Bump_Height = 0.05;            % Obstacle height [m] (50 mm)
Bump_Length = 1.0;             % Obstacle length [m]
Car_Speed_kmh = 45.0;          % Vehicle forward velocity [km/h]
Car_Speed_mps = Car_Speed_kmh / 3.6; % [m/s] (12.5 m/s)
Bump_Duration = Bump_Length / Car_Speed_mps; % [s] (0.08 s)
Bump_StartTime = 0.5;          % Excitation onset time [s]

% Chirp specifications
Chirp_Amp = 0.010;             % Amplitude [m] (10 mm)
Chirp_f0  = 0.5;               % Starting frequency [Hz]
Chirp_f1  = 15.0;              % Final frequency [Hz]
Chirp_T   = 10.0;              % Sweep duration [s]

% Step specifications
Step_Height = 0.03;            % Step height [m] (30 mm)
Step_Time   = 0.5;             % Step arrival [s]
Step_Rise   = 0.02;            % Rise time [s]

%% 5. Controller Configuration
% ControlMode:
%  -1 -> Basic Linear Passive Damper (c_s_linear = 1500 N*s/m, zeta = 0.28)
%   0 -> MR Damper - Passive Soft (0.0 V)
%   1 -> MR Damper - Passive Hard (2.0 V)
%   2 -> MR Damper - Karnopp Skyhook (2-State On-Off)
%   3 -> MR Damper - Continuous Linear Skyhook
%   4 -> MR Damper - Hybrid Skyhook-Groundhook
ControlMode = 2;

% Controller tuning gains
C_sky        = 2200.0;         % Skyhook damping coefficient [N*s/m]
F_ref_sky    = 2000.0;         % Reference force for continuous voltage scaling [N]
alpha_hybrid = 0.65;           % Skyhook vs Groundhook weighting factor (0 = pure GH, 1 = pure SH)

fprintf('Quarter-Car & MR Damper parameters initialized successfully in workspace.\n');
fprintf('  Sprung Mass:       %.1f kg (Bounce fn = %.2f Hz)\n', m_s, fn_sprung);
fprintf('  Unsprung Mass:     %.1f kg (Wheel hop fn = %.2f Hz)\n', m_u, fn_unsprung);
fprintf('  Basic Passive c_s: %.1f N*s/m (Damping ratio zeta = %.3f)\n', c_s_linear, zeta_lin);
fprintf('  Active Controller: Mode %d (-1:Basic Passive, 0:Soft, 1:Hard, 2:Skyhook-2State, 3:Cont-Skyhook, 4:Hybrid)\n', ControlMode);
