%% Ziegler-Nichols Closed-Loop Tuning Calculator
clear; clc;

% =========================================================================
% 1. ENTER YOUR EXPERIMENTAL DATA HERE
% =========================================================================
Ku = 50;  % Ultimate Gain (The P-only value that caused sustained oscillation)
Pu = 0.194;  % Ultimate Period in seconds (Time from peak-to-peak)

% =========================================================================
% 2. ZIEGLER-NICHOLS MATHEMATICAL FORMULAS
% =========================================================================
% Standard Z-N target time constants for a classic full PID controller
tau_i = 2.2 * Pu;      % Integral time constant
tau_d = Pu / 6.3;      % Derivative time constant

% --- Configuration A: Parallel Form Gains (Default Simulink PID block) ---
% Formula: Block = Kp + Ki*(1/s) + Kd*(N*s/(s+N))
Gains_Parallel.Kp = 0.3125 * Ku;
Gains_Parallel.Ki = Gains_Parallel.Kp / tau_i;
Gains_Parallel.Kd = Gains_Parallel.Kp * tau_d;

% =========================================================================
% 3. DISPLAY THE CALCULATED PARAMETERS
% =========================================================================
fprintf('====================================================\n');
fprintf('     ZIEGLER-NICHOLS PID TUNING PARAMETERS          \n');
fprintf('====================================================\n');
fprintf('Inputs Used:\n');
fprintf('  Ultimate Gain (Ku)   = %.4f\n', Ku);
fprintf('  Ultimate Period (Pu) = %.4f seconds\n\n', Pu);

fprintf('----------------------------------------------------\n');
fprintf('OPTION 1: Simulink "PARALLEL" Form Gains (Most Common)\n');
fprintf('----------------------------------------------------\n');
fprintf('  Proportional (Kp) : %7.5f\n', Gains_Parallel.Kp);
fprintf('  Integral     (Ki) : %7.5f\n', Gains_Parallel.Ki);
fprintf('  Derivative   (Kd) : %7.5f\n\n', Gains_Parallel.Kd);