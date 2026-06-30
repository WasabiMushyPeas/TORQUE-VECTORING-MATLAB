function r_ref = refgen(delta, Vx, P)
%REFGEN Steady-state bicycle-model yaw-rate reference, friction-limited.
%#codegen
    Vxs   = max(Vx, 0.5);
    r_lin = Vxs*delta / (P.L + P.Kus_ref*Vxs^2);   % steady-state bicycle yaw rate
    r_max = P.mu*9.81 / Vxs;                        % friction-limited ceiling
    r_ref = max(min(r_lin, r_max), -r_max);
    if Vx < P.Vx_min                                % low-speed fade
        r_ref = r_ref * (Vx/P.Vx_min);
    end
end
