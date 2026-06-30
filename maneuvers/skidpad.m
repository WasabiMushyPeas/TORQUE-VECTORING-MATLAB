function [t, delta] = skidpad(varargin)
%SKIDPAD Constant steering angle for steady-state cornering (skidpad proxy).
% Placeholder for Stage 0/1 use with the bicycle model. For true skidpad
% geometry/validation against FSAE rules, use Vehicle Dynamics Blockset's
% "Generate Skidpad Test" once on the double-track model (see README).
% [t, delta] = skidpad('delta', 0.08, 'duration', 6)
    p = inputParser;
    addParameter(p, 'delta', 0.08);    % rad, constant
    addParameter(p, 'duration', 6);
    addParameter(p, 'dt', 1e-3);
    parse(p, varargin{:});
    r = p.Results;

    t = (0:r.dt:r.duration)';
    delta = r.delta * ones(size(t));
end
