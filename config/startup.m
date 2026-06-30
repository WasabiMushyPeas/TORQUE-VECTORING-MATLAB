%% CP27E-TV project startup
% Runs automatically when CP27E-TV.prj is opened in MATLAB.

root = currentProject().RootFolder;

% Make sure src/ algorithm code is on the path (models call into it)
addpath(fullfile(root, 'src'));
addpath(fullfile(root, 'maneuvers'));
addpath(genpath(fullfile(root, 'scripts')));

% Load default parameters into the base workspace
run(fullfile(root, 'data', 'vehicle_params.m'));
run(fullfile(root, 'data', 'controller_params.m'));
run(fullfile(root, 'data', 'sim_config.m'));

fprintf('CP27E-TV project loaded. Parameter struct P is in the base workspace.\n');
fprintf('Run scripts/run_sim.m to simulate the default maneuver.\n');
