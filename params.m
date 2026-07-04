%% CP27E torque vectoring — all parameters
% Everything lives in struct P (car + controller + sim). SI units, ISO signs.
% Run this file (or paste into the Command Window) so P is in the workspace.
% The Simulink MATLAB Function blocks read P with "Parameter" scope.

%% ---- Vehicle: mass & geometry ----
P.m   = 278.1;            % total mass [kg]   (confirm incl. driver)
P.Izz = 110;              % yaw inertia [kg*m^2]   (estimate; measure/CAD)
P.L   = 1.55;             % wheelbase [m]     (set to CP27E)
P.t   = 1.22;             % track [m]         (set to CP27E)
P.hcg = 0.254;            % CG height [m]
P.rwf = 0.488;            % static REAR weight fraction
P.a   = P.rwf*P.L;        % CG -> front axle [m]
P.b   = (1-P.rwf)*P.L;    % CG -> rear axle  [m]
P.mf  = (1-P.rwf)*P.m;    % front axle mass [kg]
P.mr  = P.rwf*P.m;        % rear axle mass  [kg]

%% ---- Tires ----
P.Rw  = 0.203;            % loaded radius [m]
P.mu  = 1.5;              % effective friction (from TTC fit)
P.Cf  = 50000;            % front axle cornering stiffness [N/rad] (Pacejka fit)
P.Cr  = 55000;            % rear  axle cornering stiffness [N/rad] (Pacejka fit)

%% ---- Powertrain ----
P.gear     = 4.5;         % motor -> wheel reduction (confirm CP27E)
P.Tmot_pk  = 21.0;        % peak motor torque [Nm] (<=1.24 s)
P.regen_on = true;        % allow negative torque on inner wheels

%% ---- Reference generator ----
P.Kus_nat = P.mf/P.Cf - P.mr/P.Cr;   % natural understeer gradient [s^2/m]
P.Kus_ref = P.Kus_nat;               % target gradient (lower = sharper turn-in)
P.Vx_min  = 2.0;                     % fade TV below this speed [m/s]

%% ---- Controller ----
P.Kp = 50;   P.Ki = 20;   P.Kd = 0;   % yaw-rate PID (Mz per rad/s)
P.Kff = 0;                              % feedforward gain (Mz_ff = Kff*delta)
P.tv_enable = 1;                        % 0/1 TV on-off switch
P.Ts_ctrl = 0.01;                       % controller sample time [s] (100 Hz)
P.M_lag = 0.005;                        % AMK Inverter/motor lag [ms]

%% ---- Sim setup ----
P.Vx       = 10;          % constant test speed [m/s] (~36 km/h)
P.T_drive  = 50;           % drive-torque demand [Nm] (0 = pure cornering)
P.tire     = 'mf';    % 'linear' | 'mf' (simplified Magic Formula)
P.maxStep  = 1e-3;        % solver max step [s]
P.stopTime = 6;           % sim stop time [s]