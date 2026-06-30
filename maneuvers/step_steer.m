function [t, delta] = step_steer(varargin)
%STEP_STEER Step change in steering angle.
% [t, delta] = step_steer('stepTime', 1, 'finalDelta', 0.05, 'duration', 5)
    p = inputParser;
    addParameter(p, 'stepTime', 1);
    addParameter(p, 'finalDelta', 0.05);   % rad
    addParameter(p, 'duration', 5);
    addParameter(p, 'dt', 1e-3);
    parse(p, varargin{:});
    r = p.Results;

    t = (0:r.dt:r.duration)';
    delta = zeros(size(t));
    delta(t >= r.stepTime) = r.finalDelta;
end
