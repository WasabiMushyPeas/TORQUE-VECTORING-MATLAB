function [T_FL,T_FR,T_RL,T_RR, Mz_real, Fx_tot] = allocate(Mz_dem, T_drive, P)
%ALLOCATE Simple proportional Mz -> four motor torques, with saturation.
% Stage 0/1 rule-based allocator. Upgrade path: QP/WLS minimizing tire
% utilization under per-wheel friction-circle + motor constraints once on
% the double-track plant (see build guide Step 11 / README "Status").
%#codegen
    Tbase = T_drive/4;                          % per-wheel base motor torque
    dT    = Mz_dem * P.Rw / (2*P.t*P.gear);     % per-wheel torque delta for Mz
    % ISO convention: +Mz (CCW / left turn) -> MORE torque on RIGHT wheels
    T_FL = Tbase - dT;  T_RL = Tbase - dT;
    T_FR = Tbase + dT;  T_RR = Tbase + dT;

    Tlo = -P.Tmot_pk * double(P.regen_on);      % 0 if regen disabled
    Thi =  P.Tmot_pk;
    T_FL=min(max(T_FL,Tlo),Thi); T_RL=min(max(T_RL,Tlo),Thi);
    T_FR=min(max(T_FR,Tlo),Thi); T_RR=min(max(T_RR,Tlo),Thi);

    % realized yaw moment from (possibly saturated) torques
    FxFL=T_FL*P.gear/P.Rw; FxRL=T_RL*P.gear/P.Rw;
    FxFR=T_FR*P.gear/P.Rw; FxRR=T_RR*P.gear/P.Rw;
    Mz_real = (P.t/2)*((FxFR+FxRR) - (FxFL+FxRL));
    Fx_tot  = FxFL+FxFR+FxRL+FxRR;
end
