function [vy_dot, r_dot, beta, ay] = bicycle(vy, r, delta, Mz, Vx, P)
%BICYCLE Single-track (bicycle) plant with direct yaw-moment input.
% Stage 0/1: linear tire model. Swap Fyf/Fyr for mf_tire() to go nonlinear
% (Stage 2 of the build guide) once a Pacejka fit is available.
%#codegen
    Vxs = max(Vx, 0.5);                 % guard /0
    af = delta - (vy + P.a*r)/Vxs;      % front slip angle
    ar =       - (vy - P.b*r)/Vxs;      % rear  slip angle

    % --- LINEAR tire (Stage 0/1) ---
    Fyf = P.Cf*af;
    Fyr = P.Cr*ar;

    vy_dot = (Fyf + Fyr)/P.m - Vx*r;        % lateral dynamics (constant Vx)
    r_dot  = (P.a*Fyf - P.b*Fyr + Mz)/P.Izz;% yaw dynamics with direct Mz
    beta   = atan2(vy, Vxs);
    ay     = (Fyf + Fyr)/P.m;
end
