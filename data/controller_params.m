%% CP27E torque-vectoring controller parameters
% Requires P (vehicle params) already loaded. Adds to struct P.

%% --- Reference generator ---
P.Kus_nat = P.mf/P.Cf - P.mr/P.Cr;   % natural understeer gradient [s^2/m]
P.Kus_ref = P.Kus_nat;               % target gradient (lower = sharper turn-in)
P.Vx_min  = 2.0;                     % fade TV below this speed [m/s]

%% --- Controller gains (tune via scripts/sweep_gains.m) ---
P.Kp  = 300;   % yaw-rate PID, Mz per (rad/s)
P.Ki  = 300;
P.Kd  = 0;
P.Kff = 0;     % feedforward gain: Mz_ff = Kff*delta (add after PID tuned)

P.tv_enable = 1;   % 0/1 switch for A/B testing TV on vs off
