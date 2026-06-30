%% run_sim.m — single entry point to simulate CP27E-TV
% Requires the CP27E-TV project to be open (so P / simCfg are loaded and
% src/, maneuvers/ are on the path). See config/startup.m.

%% --- Choose a maneuver ---
maneuverName = 'ramp_to_limit';   % 'step_steer' | 'ramp_to_limit' | 'skidpad'

switch maneuverName
    case 'step_steer'
        [t_in, delta_in] = step_steer('finalDelta', 0.05);
    case 'ramp_to_limit'
        [t_in, delta_in] = ramp_to_limit('finalDelta', 0.30);
    case 'skidpad'
        [t_in, delta_in] = skidpad('delta', 0.08);
    otherwise
        error('Unknown maneuver: %s', maneuverName);
end

deltaInput = timeseries(delta_in, t_in);

%% --- A/B test: TV on vs off ---
modelName = 'cp27e_tv_top';   % top-level harness model in models/
if ~bdIsLoaded(modelName)
    open_system(fullfile(currentProject().RootFolder, 'models', [modelName '.slx']));
end

results = struct();
for tv = [0 1]
    P.tv_enable = tv; %#ok<NASGU>  % picked up by the model via base workspace
    set_param(modelName, 'StopTime', num2str(simCfg.stopTime));
    simOut = sim(modelName);

    key = sprintf('tv%d', tv);
    results.(key).t       = simOut.tout;
    results.(key).r       = simOut.logsout.get('r').Values.Data;
    results.(key).r_ref   = simOut.logsout.get('r_ref').Values.Data;
    results.(key).ay      = simOut.logsout.get('ay').Values.Data;
end

%% --- Plot ---
figure('Name', 'CP27E Torque Vectoring — TV off vs on');
subplot(2,1,1); hold on; grid on;
plot(results.tv0.t, results.tv0.r, 'b--', 'DisplayName', 'r, TV off');
plot(results.tv1.t, results.tv1.r, 'b-',  'DisplayName', 'r, TV on');
plot(results.tv1.t, results.tv1.r_ref, 'k:', 'DisplayName', 'r_{ref}');
xlabel('Time [s]'); ylabel('Yaw rate [rad/s]'); legend; title('Yaw rate tracking');

subplot(2,1,2); hold on; grid on;
plot(results.tv0.t, results.tv0.ay, 'r--', 'DisplayName', 'a_y, TV off');
plot(results.tv1.t, results.tv1.ay, 'r-',  'DisplayName', 'a_y, TV on');
xlabel('Time [s]'); ylabel('Lateral accel [m/s^2]'); legend; title('Lateral acceleration');

% NOTE: this script assumes the model logs signals named 'r', 'r_ref',
% 'ay' via Simulink logsout (right-click signal -> Log Selected Signal).
% Adjust signal names here if your model uses different ones.
