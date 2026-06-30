function Fy = mf_tire(alpha, Fz, mu, B, C)
%MF_TIRE Simplified (lumped, pure-lateral-slip) Magic Formula tire.
% Fy = D*sin(C*atan(B*alpha)), D = mu*Fz.
% Stiffness factor B should be chosen so that B*C*D matches your linear
% cornering stiffness at alpha=0, for continuity with bicycle.m's linear
% model. Replace with a full combined-slip Pacejka fit from TTC data
% (see Teasdale's Magic Formula Tyre Tool) when available.
%#codegen
    D  = mu*Fz;
    Fy = D*sin(C*atan(B*alpha));
end
