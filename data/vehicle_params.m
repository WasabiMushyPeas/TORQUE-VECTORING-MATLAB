%% CP27E vehicle parameters
% SI units, ISO sign convention. Populates struct P in base workspace.
% Items marked <-- need confirmation/measurement for CP27E.

%% --- Mass & geometry ---
P.m   = 278.1;            % total mass [kg]   <-- confirm incl. driver
P.Izz = 110;              % yaw inertia [kg*m^2]  <-- ESTIMATE; measure (bifilar) or CAD
P.L   = 1.55;             % wheelbase [m]     <-- set to CP27E
P.t   = 1.22;             % track [m]         <-- set to CP27E
P.hcg = 0.254;            % CG height [m]
P.rwf = 0.488;            % static REAR weight fraction
P.a   = P.rwf*P.L;        % CG -> front axle [m]
P.b   = (1-P.rwf)*P.L;    % CG -> rear axle  [m]
P.mf  = (1-P.rwf)*P.m;    % front axle mass [kg]
P.mr  = P.rwf*P.m;        % rear axle mass  [kg]

%% --- Tires ---
P.Rw  = 0.203;            % loaded radius [m]
P.mu  = 1.5;              % effective friction  <-- from TTC fit
P.Cf  = 50000;            % front AXLE cornering stiffness [N/rad]  <-- REPLACE w/ Pacejka fit
P.Cr  = 55000;            % rear  AXLE cornering stiffness [N/rad]  <-- REPLACE w/ Pacejka fit

%% --- Powertrain (AMK DD5-14-10-POW x4) ---
P.gear    = 4.5;          % motor -> wheel reduction  <-- confirm CP27E ratio
P.Tmot_pk = 21.0;         % peak motor torque [Nm] (<=1.24 s)
P.regen_on = true;        % allow negative torque on inner wheels
