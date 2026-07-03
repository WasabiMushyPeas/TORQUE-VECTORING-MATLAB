function [vy_dot, r_dot, beta, ay] = bicycle(vy, r, delta, Mz, Vx, P)

Vxs = max(Vx, 0.5);
af = delta - (vy + P.a*r)/Vxs;     % front slip angle
ar =       - (vy - P.b*r)/Vxs;     % rear  slip angle

if isfield(P,'tire') && strcmp(P.tire,'mf')
    Fzf=P.mf*9.81; Fzr=P.mr*9.81; Cmf=1.4;
    Bf=P.Cf/(Cmf*P.mu*Fzf); Br=P.Cr/(Cmf*P.mu*Fzr);
    Fyf=mf_tire(af,Fzf,P.mu,Bf,Cmf);
    Fyr=mf_tire(ar,Fzr,P.mu,Br,Cmf);
else
    Fyf = P.Cf*af;                 % linear tire
    Fyr = P.Cr*ar;
end

vy_dot = (Fyf + Fyr)/P.m - Vx*r;              % lateral dynamics
r_dot  = (P.a*Fyf - P.b*Fyr + Mz)/P.Izz;      % yaw dynamics (+Mz from TV)
beta   = atan2(vy, Vxs);
ay     = (Fyf + Fyr)/P.m;
end