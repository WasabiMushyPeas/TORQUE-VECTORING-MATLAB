function [vy_dot, r_dot, Vx_dot, beta, ay, ax, Fz] = doubletrack(vy, r, Vx, delta, T_FL, T_FR, T_RL, T_RR, ax_f, ay_f, P)
    g = 9.81;
    Vxs = max(Vx, 0.5);
    ht  = P.t/2;
    
    %% --- Per-wheel vertical loads (static + longitudinal + lateral transfer) ---
    % Static
    Fz_FL = P.mf*g/2;  Fz_FR = P.mf*g/2;
    Fz_RL = P.mr*g/2;  Fz_RR = P.mr*g/2;
    
    % Longitudinal transfer: accel unloads front, loads rear
    dFz_long = P.m*ax_f*P.hcg/P.L / 2;          % per wheel
    Fz_FL = Fz_FL - dFz_long;  Fz_FR = Fz_FR - dFz_long;
    Fz_RL = Fz_RL + dFz_long;  Fz_RR = Fz_RR + dFz_long;
    
    % Lateral transfer: +ay (left turn) loads the RIGHT side.
    % Total dFz = m*ay*hcg/t, split front/rear by roll stiffness fraction.
    dFz_lat  = P.m*ay_f*P.hcg/P.t;
    Fz_FL = Fz_FL - P.rsf*dFz_lat;      Fz_FR = Fz_FR + P.rsf*dFz_lat;
    Fz_RL = Fz_RL - (1-P.rsf)*dFz_lat;  Fz_RR = Fz_RR + (1-P.rsf)*dFz_lat;
    
    % No negative loads (wheel lift), keep tiny minimum for numerics
    Fz_FL=max(Fz_FL,10); Fz_FR=max(Fz_FR,10);
    Fz_RL=max(Fz_RL,10); Fz_RR=max(Fz_RR,10);
    Fz = [Fz_FL; Fz_FR; Fz_RL; Fz_RR];
    
    %% --- Per-wheel slip angles (each corner sees its own velocity) ---
    a_FL = delta - (vy + P.a*r)/max(Vxs - r*ht, 0.5);
    a_FR = delta - (vy + P.a*r)/max(Vxs + r*ht, 0.5);
    a_RL =       - (vy - P.b*r)/max(Vxs - r*ht, 0.5);
    a_RR =       - (vy - P.b*r)/max(Vxs + r*ht, 0.5);
    
    %% --- Longitudinal tire forces from motor torques, friction-capped ---
    Fx_FL = T_FL*P.gear/P.Rw;  Fx_FR = T_FR*P.gear/P.Rw;
    Fx_RL = T_RL*P.gear/P.Rw;  Fx_RR = T_RR*P.gear/P.Rw;
    Fx_FL = min(max(Fx_FL, -P.mu*Fz_FL), P.mu*Fz_FL);
    Fx_FR = min(max(Fx_FR, -P.mu*Fz_FR), P.mu*Fz_FR);
    Fx_RL = min(max(Fx_RL, -P.mu*Fz_RL), P.mu*Fz_RL);
    Fx_RR = min(max(Fx_RR, -P.mu*Fz_RR), P.mu*Fz_RR);
    
    %% --- Lateral tire forces: load-sensitive MF x friction-ellipse derate ---
    % Per-wheel stiffness factor from HALF the axle stiffness at static load,
    % so slope scales with Fz (simple load sensitivity).
    Cmf = 1.4;
    Bf  = (P.Cf/2)/(Cmf*P.mu*(P.mf*g/2));
    Br  = (P.Cr/2)/(Cmf*P.mu*(P.mr*g/2));
    
    Fy_FL = P.mu*Fz_FL*sin(Cmf*atan(Bf*a_FL));
    Fy_FR = P.mu*Fz_FR*sin(Cmf*atan(Bf*a_FR));
    Fy_RL = P.mu*Fz_RL*sin(Cmf*atan(Br*a_RL));
    Fy_RR = P.mu*Fz_RR*sin(Cmf*atan(Br*a_RR));
    
    % Friction ellipse: longitudinal use eats lateral capacity
    Fy_FL = Fy_FL*sqrt(max(0, 1-(Fx_FL/(P.mu*Fz_FL))^2));
    Fy_FR = Fy_FR*sqrt(max(0, 1-(Fx_FR/(P.mu*Fz_FR))^2));
    Fy_RL = Fy_RL*sqrt(max(0, 1-(Fx_RL/(P.mu*Fz_RL))^2));
    Fy_RR = Fy_RR*sqrt(max(0, 1-(Fx_RR/(P.mu*Fz_RR))^2));
    
    %% --- Sum forces & moments (small steer angle: cos(delta)~1) ---
    Fy_sum = Fy_FL + Fy_FR + Fy_RL + Fy_RR;
    Fx_sum = Fx_FL + Fx_FR + Fx_RL + Fx_RR;
    
    Fdrag = 0.5*P.rho*P.CdA*Vx^2 + P.Crr*P.m*g;
    
    Mz = P.a*(Fy_FL+Fy_FR) - P.b*(Fy_RL+Fy_RR) ...     % lateral forces
        + ht*((Fx_FR+Fx_RR) - (Fx_FL+Fx_RL));           % differential drive = TV
    
    %% --- Equations of motion ---
    vy_dot = Fy_sum/P.m - Vx*r;
    r_dot  = Mz/P.Izz;
    Vx_dot = (Fx_sum - Fdrag)/P.m + r*vy;
    ax     = (Fx_sum - Fdrag)/P.m;          % body-frame, for load transfer
    ay     = Fy_sum/P.m;
    beta   = atan2(vy, Vxs);
end