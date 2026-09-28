%% Standalone Pure MATLAB Simulation of Quarter-Car with MR Damper
% Runs without requiring Simulink. Solves the stiff nonlinear system using ode15s.
%
% Architectures Modeled:
%   (a) Passive Quarter-Car:
%       m_s * d2(z_s)/dt^2 + c_s * (d(z_s)/dt - d(z_u)/dt) + k_s * (z_s - z_u) = 0
%       m_u * d2(z_u)/dt^2 - c_s * (d(z_s)/dt - d(z_u)/dt) - k_s * (z_s - z_u) + k_t * (z_u - z_r) = 0
%   (b) Semi-Active Quarter-Car:
%       m_s * d2(z_s)/dt^2 + k_s * (z_s - z_u) + F_d = 0
%       m_u * d2(z_u)/dt^2 - k_s * (z_s - z_u) - F_d + k_t * (z_u - z_r) = 0
%       where F_d is the controllable damping force (Spencer Modified Bouc-Wen MR damper).
%
% Author: W. M. Baig
%
% Citation Request:
%   [1] Z. Yu, R. Luo, P. Wu, W. M. Baig, H. Ma, Z. Hou, IEEE TTE 2025 (DOI: 10.1109/TTE.2025.3535765)
%   [2] W. M. Baig, Z. Yu, H. Ma, Z. Hou, IEEE VTC2025-Spring (DOI: 10.1109/VTC2025-Spring65109.2025.11174543)
%   [3] W. M. Baig, Z. Hou, S. Ijaz, CCDC 2017 (pp. 2808-2813)

clear; clc; close all;

%% 1. Load Parameters
run('init_quarter_car_params.m');

%% 2. Define Road Profile Function
road_eval = @(t) evaluate_road(t, RoadType, Bump_Height, Bump_Length, Car_Speed_mps, Bump_StartTime);

%% 3. Benchmark Controllers
controllers = {
    struct('name', 'Basic Passive (Linear)', 'mode', -1, 'color', [0.45, 0.45, 0.45]), ...
    struct('name', 'Passive Soft (0.0 V)',   'mode', 0,  'color', [0.12, 0.47, 0.71]), ...
    struct('name', 'Passive Hard (2.0 V)',   'mode', 1,  'color', [0.84, 0.15, 0.16]), ...
    struct('name', 'Skyhook (2-State)',      'mode', 2,  'color', [0.17, 0.63, 0.17]), ...
    struct('name', 'Continuous Skyhook',     'mode', 3,  'color', [1.00, 0.50, 0.05]), ...
    struct('name', 'Hybrid Sky-Ground',      'mode', 4,  'color', [0.58, 0.40, 0.74])
};

results = cell(length(controllers), 1);
t_span = [0, SimTime];
init_state = zeros(7, 1); % [z_s, z_s_dot, z_u, z_u_dot, y, z, u]

fprintf('\nRunning Quarter-Car Stiff ODE Simulation (ode15s)...\n');

for c_idx = 1:length(controllers)
    ctrl = controllers{c_idx};
    fprintf('  Simulating %-25s...', ctrl.name);
    
    % ODE RHS function
    ode_fun = @(t, x) quarter_car_ode(t, x, ctrl.mode, road_eval, ...
        m_s, m_u, k_s, c_s_pass, c_s_linear, k_t, c_t, ...
        c0_a, c0_b, k0, c1_a, c1_b, k1, x0, alpha_a, alpha_b, gamma, beta, A, n, eta, ...
        V_min, V_max, C_sky, F_ref_sky, alpha_hybrid);
    
    options = odeset('RelTol', 1e-5, 'AbsTol', 1e-8, 'MaxStep', 1e-3);
    [t_out, x_out] = ode15s(ode_fun, t_span, init_state, options);
    
    % Post-process signals
    N = length(t_out);
    z_s = x_out(:, 1);
    z_s_dot = x_out(:, 2);
    z_u = x_out(:, 3);
    z_u_dot = x_out(:, 4);
    y_d = x_out(:, 5);
    z_d = x_out(:, 6);
    u_d = x_out(:, 7);
    
    susp_deflection = z_s - z_u;
    susp_velocity = z_s_dot - z_u_dot;
    
    f_mr = zeros(N, 1);
    z_s_ddot = zeros(N, 1);
    tire_deflection = zeros(N, 1);
    zr_arr = zeros(N, 1);
    
    for i = 1:N
        [zr, ~] = road_eval(t_out(i));
        zr_arr(i) = zr;
        tire_deflection(i) = z_u(i) - zr;
        
        if ctrl.mode == -1
            f_mr(i) = c_s_linear * susp_velocity(i);
        else
            c0 = c0_a + c0_b * u_d(i);
            c1 = c1_a + c1_b * u_d(i);
            alpha = alpha_a + alpha_b * u_d(i);
            y_dot = (alpha * z_d(i) + c0 * susp_velocity(i) + k0 * (susp_deflection(i) - y_d(i))) / (c0 + c1);
            f_mr(i) = c1 * y_dot + k1 * (susp_deflection(i) - x0);
        end
        
        f_spring = k_s * susp_deflection(i) + c_s_pass * susp_velocity(i);
        z_s_ddot(i) = (-f_spring - f_mr(i)) / m_s;
    end
    
    % Metrics
    rms_accel = sqrt(mean(z_s_ddot.^2));
    peak_disp = max(abs(z_s)) * 1000; % mm
    peak_travel = max(abs(susp_deflection)) * 1000; % mm
    
    results{c_idx} = struct('name', ctrl.name, 'color', ctrl.color, ...
        't', t_out, 'z_s', z_s, 'z_s_ddot', z_s_ddot, 'susp_defl', susp_deflection, ...
        'f_mr', f_mr, 'tire_defl', tire_deflection, 'zr', zr_arr, ...
        'rms_accel', rms_accel, 'peak_disp', peak_disp, 'peak_travel', peak_travel);
    
    fprintf(' Done. Peak Disp = %.1f mm, RMS Accel = %.2f m/s^2\n', peak_disp, rms_accel);
end

%% 4. Print Summary Table
fprintf('\n========================================================================\n');
fprintf('%-24s | %-16s | %-14s | %-16s\n', 'Controller', 'RMS Accel [m/s^2]', 'Peak Disp [mm]', 'Peak Travel [mm]');
fprintf('------------------------------------------------------------------------\n');
for i = 1:length(results)
    r = results{i};
    fprintf('%-24s | %-16.2f | %-14.1f | %-16.1f\n', r.name, r.rms_accel, r.peak_disp, r.peak_travel);
end
fprintf('========================================================================\n');

%% 5. Plot Comparison Figures
figure('Name', 'Quarter-Car Semi-Active Suspension Benchmark', 'Position', [100, 100, 1000, 850]);

subplot(4, 1, 1);
plot(results{1}.t, results{1}.zr * 1000, 'k--', 'LineWidth', 1.2, 'DisplayName', 'Road Profile');
hold on;
for i = 1:length(results)
    plot(results{i}.t, results{i}.z_s * 1000, 'Color', results{i}.color, 'LineWidth', 1.8, 'DisplayName', results{i}.name);
end
ylabel('Body Disp. [mm]', 'FontWeight', 'bold');
title('Sprung Mass Vertical Displacement (Ride Comfort)', 'FontWeight', 'bold');
grid on; legend('Location', 'northeast');

subplot(4, 1, 2);
hold on;
for i = 1:length(results)
    plot(results{i}.t, results{i}.z_s_ddot, 'Color', results{i}.color, 'LineWidth', 1.8);
end
ylabel('Body Accel. [m/s^2]', 'FontWeight', 'bold');
title('Sprung Mass Vertical Acceleration (ISO 2631)', 'FontWeight', 'bold');
grid on;

subplot(4, 1, 3);
hold on;
for i = 1:length(results)
    plot(results{i}.t, results{i}.susp_defl * 1000, 'Color', results{i}.color, 'LineWidth', 1.8);
end
ylabel('Deflection [mm]', 'FontWeight', 'bold');
title('Suspension Working Space (z_s - z_u)', 'FontWeight', 'bold');
grid on;

subplot(4, 1, 4);
hold on;
for i = 1:length(results)
    plot(results{i}.t, results{i}.f_mr, 'Color', results{i}.color, 'LineWidth', 1.8);
end
ylabel('Force [N]', 'FontWeight', 'bold');
xlabel('Time [s]', 'FontWeight', 'bold');
title('Magnetorheological Damper Output Force (F_{MR})', 'FontWeight', 'bold');
grid on;

%% Helper: Road Evaluation Function
function [zr, zr_dot] = evaluate_road(t, type, H, L, v, t_start)
    t_end = t_start + L / v;
    if type == 1 && t >= t_start && t <= t_end
        tau = (t - t_start) / (L / v);
        zr = 0.5 * H * (1 - cos(2 * pi * tau));
        zr_dot = 0.5 * H * (2 * pi / (L / v)) * sin(2 * pi * tau);
    else
        zr = 0.0;
        zr_dot = 0.0;
    end
end

%% Helper: Quarter-Car ODE RHS Function
function dxdt = quarter_car_ode(t, x, mode, road_eval, ...
    m_s, m_u, k_s, c_s_pass, c_s_linear, k_t, c_t, ...
    c0_a, c0_b, k0, c1_a, c1_b, k1, x0, alpha_a, alpha_b, gamma, beta, A, n, eta, ...
    V_min, V_max, C_sky, F_ref_sky, alpha_hybrid)

    z_s     = x(1);
    z_s_dot = x(2);
    z_u     = x(3);
    z_u_dot = x(4);
    y_d     = x(5);
    z_d     = x(6);
    u_d     = x(7);

    [zr, zr_dot] = road_eval(t);

    susp_defl = z_s - z_u;
    susp_vel  = z_s_dot - z_u_dot;

    if mode == -1
        % Standard linear passive shock absorber: F_d = c_s_linear * x_dot
        f_mr = c_s_linear * susp_vel;
        f_spring = k_s * susp_defl + c_s_pass * susp_vel;
        f_tire   = k_t * (z_u - zr) + c_t * (z_u_dot - zr_dot);
        dz_s_ddot = (-f_spring - f_mr) / m_s;
        dz_u_ddot = (f_spring + f_mr - f_tire) / m_u;
        dxdt = [z_s_dot; dz_s_ddot; z_u_dot; dz_u_ddot; 0; 0; 0];
        return;
    end

    % Controller logic
    switch mode
        case 0  % Passive Soft
            v_cmd = 0.0;
        case 1  % Passive Hard
            v_cmd = 2.0;
        case 2  % Skyhook 2-State
            if z_s_dot * susp_vel >= 0
                v_cmd = V_max;
            else
                v_cmd = V_min;
            end
        case 3  % Continuous Skyhook
            if z_s_dot * susp_vel >= 0
                f_des = C_sky * abs(z_s_dot);
                v_cmd = min(V_max, max(V_min, V_min + (f_des / F_ref_sky) * (V_max - V_min)));
            else
                v_cmd = V_min;
            end
        case 4  % Hybrid Skyhook-Groundhook
            sigma = alpha_hybrid * z_s_dot - (1 - alpha_hybrid) * z_u_dot;
            if sigma * susp_vel >= 0
                v_cmd = V_max;
            else
                v_cmd = V_min;
            end
        otherwise
            v_cmd = 0.0;
    end

    v_cmd = min(V_max, max(V_min, v_cmd));

    % Coil dynamics
    du_dt = -eta * (u_d - v_cmd);

    % Voltage dependent parameters
    alpha = alpha_a + alpha_b * u_d;
    c0    = c0_a + c0_b * u_d;
    c1    = c1_a + c1_b * u_d;

    % Intermediate float plate velocity
    dy_dt = (alpha * z_d + c0 * susp_vel + k0 * (susp_defl - y_d)) / (c0 + c1);

    % Evolutionary Bouc-Wen rate
    vel_rel = susp_vel - dy_dt;
    abs_z = abs(z_d);
    z_n = abs_z^n;
    if abs_z > 1e-18
        z_n_minus_1 = abs_z^(n - 1);
    else
        z_n_minus_1 = 0.0;
    end
    dz_dt = A * vel_rel - beta * vel_rel * z_n - gamma * abs(vel_rel) * z_n_minus_1 * z_d;

    % Total MR damper force
    f_mr = c1 * dy_dt + k1 * (susp_defl - x0);

    % Forces on masses
    f_spring = k_s * susp_defl + c_s_pass * susp_vel;
    f_tire   = k_t * (z_u - zr) + c_t * (z_u_dot - zr_dot);

    dz_s_ddot = (-f_spring - f_mr) / m_s;
    dz_u_ddot = (f_spring + f_mr - f_tire) / m_u;

    dxdt = [z_s_dot; dz_s_ddot; z_u_dot; dz_u_ddot; dy_dt; dz_dt; du_dt];
end
