%% CP27E simulation configuration
% Solver and default operating-point settings.

%% --- Operating point ---
P.Vx = 10;   % constant test speed [m/s] (~36 km/h) for Stage 0/1 sims

%% --- Solver (Stage 0/1: variable-step while validating the plant) ---
simCfg.solver      = 'ode45';
simCfg.maxStep     = 1e-3;
simCfg.stopTime    = 6;       % seconds

% Once closed-loop behavior looks right, switch to fixed-step for the
% auto-code path:
%   simCfg.solver  = 'FixedStepDiscrete' (ode4), step = 1e-3 (plant)
%   controller sample time = 0.01 (100 Hz, matches target VCU loop rate)
