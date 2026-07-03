function [T_FL,T_FR,T_RL,T_RR, Mz_real, Fx_tot] = allocate(Mz_dem, T_drive, P)
    Tbase = T_drive/4;
    dT    = Mz_dem * P.Rw / (2*P.t*P.gear);       % per-wheel torque delta
    T_FL=Tbase-dT; T_RL=Tbase-dT; T_FR=Tbase+dT; T_RR=Tbase+dT;

    Tlo = -P.Tmot_pk*double(P.regen_on); Thi = P.Tmot_pk;
    T_FL=min(max(T_FL,Tlo),Thi); T_RL=min(max(T_RL,Tlo),Thi);
    T_FR=min(max(T_FR,Tlo),Thi); T_RR=min(max(T_RR,Tlo),Thi);

    FxFL=T_FL*P.gear/P.Rw; FxRL=T_RL*P.gear/P.Rw;
    FxFR=T_FR*P.gear/P.Rw; FxRR=T_RR*P.gear/P.Rw;
    Mz_real=(P.t/2)*((FxFR+FxRR)-(FxFL+FxRL));    % realized yaw moment
    Fx_tot = FxFL+FxFR+FxRL+FxRR;
end