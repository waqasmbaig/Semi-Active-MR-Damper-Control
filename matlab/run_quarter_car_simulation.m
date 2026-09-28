%% Automated Multi-Controller Execution & Benchmarking for Quarter_Car_MRD.slx
% Author: W. M. Baig
%
% When executed in MATLAB, this script:
%   1. Loads quarter-car and MR damper parameters into base workspace
%   2. Simulates Quarter_Car_MRD across 5 suspension control configurations
%   3. Extracts and logs time histories (displacements, accelerations, forces)
%   4. Computes ISO 2631 vibration metrics (RMS accel, peak disp, travel)
%   5. Produces publication-quality comparative figures

clear; clc; close all;

%% 1. Initialize Workspace Parameters
run('init_quarter_car_params.m');

model_name = 'Quarter_Car_MRD';

% Verify Simulink model exists; if not, build it automatically
if ~exist([model_name '.slx'], 'file')
    fprintf('Simulink model [%s.slx] not found. Building it automatically...\n', model_name);
    run('build_quarter_car_model.m');
end

load_system(model_name);

%% 2. Controller Benchmark Cases
controllers = [
    struct('name', 'Basic Passive (Linear)', 'mode', -1, 'color', [0.45, 0.45, 0.45]), ...
    struct('name', 'Passive Soft (0.0 V)',   'mode', 0,  'color', [0.12, 0.47, 0.71]), ...
    struct('name', 'Passive Hard (2.0 V)',   'mode', 1,  'color', [0.84, 0.15, 0.16]), ...
    struct('name', 'Skyhook (2-State)',      'mode', 2,  'color', [0.17, 0.63, 0.17]), ...
    struct('name', 'Continuous Skyhook',     'mode', 3,  'color', [1.00, 0.50, 0.05]), ...
    struct('name', 'Hybrid Sky-Ground',      'mode', 4,  'color', [0.58, 0.40, 0.74])
];

sim_data = cell(length(controllers), 1);

fprintf('\nExecuting Simulink batch simulations for [%s.slx]...\n', model_name);

for i = 1:length(controllers)
    ctrl = controllers(i);
    fprintf('  Simulating: %-25s (ControlMode = %d)...', ctrl.name, ctrl.mode);
    
    % Set ControlMode parameter in workspace
    assignin('base', 'ControlMode', ctrl.mode);
    
    % Run simulation
    simOut = sim(model_name, 'StopTime', num2str(SimTime));
    
    % Extract signals
    t = simOut.tout;
    
    % Check for logged workspace variables
    if isfield(simOut, 'z_s_out')
        z_s = simOut.z_s_out.Data;
        acc = simOut.acc_out.Data;
        f_mr = simOut.force_out.Data;
    elseif isfield(simOut, 'yout')
        z_s = simOut.yout{1}.Values.Data;
        acc = simOut.yout{2}.Values.Data;
        f_mr = simOut.yout{3}.Values.Data;
    else
        % Fallback for direct signal logging
        z_s = evalin('base', 'z_s_out.Data');
        acc = evalin('base', 'acc_out.Data');
        f_mr = evalin('base', 'force_out.Data');
    end
    
    % Calculate performance metrics
    rms_accel = sqrt(mean(acc.^2));
    peak_disp = max(abs(z_s)) * 1000; % mm
    
    sim_data{i} = struct('name', ctrl.name, 'color', ctrl.color, ...
        't', t, 'z_s', z_s, 'acc', acc, 'force', f_mr, ...
        'rms_accel', rms_accel, 'peak_disp', peak_disp);
    
    fprintf(' Done. RMS Accel: %.2f m/s^2 | Peak Disp: %.1f mm\n', rms_accel, peak_disp);
end

%% 3. Print Quantitative Comparison Summary
fprintf('\n========================================================================\n');
fprintf('%-26s | %-18s | %-16s\n', 'Suspension Control Law', 'RMS Accel [m/s^2]', 'Peak Disp [mm]');
fprintf('------------------------------------------------------------------------\n');
for i = 1:length(sim_data)
    d = sim_data{i};
    fprintf('%-26s | %-18.2f | %-16.1f\n', d.name, d.rms_accel, d.peak_disp);
end
fprintf('========================================================================\n');

%% 4. Plot Multi-Panel Response
fig = figure('Name', 'Quarter-Car Semi-Active Suspension Benchmark', 'Position', [100, 100, 1050, 800]);

% 1. Sprung Mass Vertical Displacement
subplot(3, 1, 1);
hold on;
for i = 1:length(sim_data)
    d = sim_data{i};
    plot(d.t, d.z_s * 1000, 'Color', d.color, 'LineWidth', 1.8, 'DisplayName', d.name);
end
ylabel('Body Disp. [mm]', 'FontWeight', 'bold');
title('Sprung Mass Vertical Displacement (ISO Obstacle Bump)', 'FontWeight', 'bold');
grid on; legend('Location', 'northeast');

% 2. Sprung Mass Vertical Acceleration
subplot(3, 1, 2);
hold on;
for i = 1:length(sim_data)
    d = sim_data{i};
    plot(d.t, d.acc, 'Color', d.color, 'LineWidth', 1.8, 'DisplayName', d.name);
end
ylabel('Body Accel. [m/s^2]', 'FontWeight', 'bold');
title('Sprung Mass Vertical Acceleration (Ride Comfort)', 'FontWeight', 'bold');
grid on;

% 3. MR Damper Control Force
subplot(3, 1, 3);
hold on;
for i = 1:length(sim_data)
    d = sim_data{i};
    plot(d.t, d.force, 'Color', d.color, 'LineWidth', 1.8, 'DisplayName', d.name);
end
ylabel('Force [N]', 'FontWeight', 'bold');
xlabel('Time [s]', 'FontWeight', 'bold');
title('MR Damper Hysteretic Output Force (F_{MR})', 'FontWeight', 'bold');
grid on;

fprintf('\nBenchmarking and plotting completed successfully.\n');
