function [t, delta] = ramp_to_limit(varargin)
%RAMP_TO_LIMIT Steering ramp from 0 to a final angle, then hold.
% Use to find the vehicle's limit / understeer behavior (build guide Step 9).
% [t, delta] = ramp_to_limit('rampDuration', 5, 'finalDelta', 0.30, 'duration', 6)
    p = inputParser;
    addParameter(p, 'rampDuration', 5);
    addParameter(p, 'finalDelta', 0.30);   % rad
    addParameter(p, 'duration', 6);
    addParameter(p, 'dt', 1e-3);
    parse(p, varargin{:});
    r = p.Results;

    t = (0:r.dt:r.duration)';
    delta = min(t, r.rampDuration) / r.rampDuration * r.finalDelta;
end
