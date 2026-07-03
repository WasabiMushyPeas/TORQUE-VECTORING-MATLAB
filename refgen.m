function r_ref = refgen(delta, Vx, P)
    Vxs   = max(Vx, 0.1);
    r_lin = Vxs*delta / (P.L + P.Kus_ref*Vxs^2);   % steady-state yaw rate
    r_max = P.mu*9.81 / Vxs;                        % friction ceiling
    r_ref = max(min(r_lin, r_max), -r_max);
    if Vx < P.Vx_min
        r_ref = r_ref * (Vx/P.Vx_min);             % low-speed fade
    end
end