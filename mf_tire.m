function Fy = mf_tire(alpha, Fz, mu, B, C)
    D  = mu*Fz;
    Fy = D*sin(C*atan(B*alpha));
end