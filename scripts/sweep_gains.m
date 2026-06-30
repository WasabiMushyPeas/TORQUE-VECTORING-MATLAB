%% sweep_gains.m — simple grid sweep over PID gains
% Requires the CP27E-TV project to be open. Sweeps Kp/Ki at fixed Kd and
% reports a basic tracking-error metric for each combination. Starting
% point for tuning per build-guide Step 10; refine the cost metric
% (overshoot, settling time, etc.) as needed.

modelName = 'cp27e_tv_top';
if ~bdIsLoaded(modelName)
    open_system(fullfile(currentProject().RootFolder, 'models', [modelName '.slx']));
end

[t_in, delta_in] = step_steer('finalDelta', 0.05); %#ok<ASGLU>
deltaInput = timeseries(delta_in, t_in); %#ok<NASGU>

KpList = [100 200 300 400];
KiList = [100 200 300 400];

cost = nan(numel(KpList), numel(KiList));

for i = 1:numel(KpList)
    for j = 1:numel(KiList)
        P.Kp = KpList(i); %#ok<NASGU>
        P.Ki = KiList(j); %#ok<NASGU>
        set_param(modelName, 'StopTime', num2str(simCfg.stopTime));
        simOut = sim(modelName);

        r     = simOut.logsout.get('r').Values.Data;
        r_ref = simOut.logsout.get('r_ref').Values.Data;
        cost(i,j) = sum((r_ref - r).^2);   % simple integral-squared-error
    end
end

figure('Name', 'Gain sweep — ISE cost');
imagesc(KiList, KpList, cost); set(gca, 'YDir', 'normal');
xlabel('Ki'); ylabel('Kp'); colorbar; title('Yaw-rate tracking ISE cost');

[~, idx] = min(cost(:));
[bi, bj] = ind2sub(size(cost), idx);
fprintf('Best: Kp=%g, Ki=%g (cost=%g)\n', KpList(bi), KiList(bj), cost(bi,bj));
