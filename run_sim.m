params;

mode = 'skidpad';          % 'step' | 'ramp' | 'steplimit' | 'skidpad'
dt = 1e-3;
switch mode
    case 'step'                                    % gentle step (verification)
        t=(0:dt:5)'; delta=zeros(size(t)); delta(t>=1)=0.05;
    case 'ramp'                                    % sweep up to the limit
        t=(0:dt:6)'; delta=min(t,5)/5*0.30;
    case 'steplimit'                               % step steer near the limit
        t=(0:dt:5)'; delta=zeros(size(t)); delta(t>=1)=0.18;
    case 'skidpad'                                 % steady constant-radius corner
        t=(0:dt:6)'; delta=0.20*ones(size(t));
    otherwise
        error('Unknown mode: %s', mode);
end
deltaInput = timeseries(delta, t);   

model='CP27E_TV';
if ~bdIsLoaded(model), open_system(model); end
R=struct();

for tv=[0 1]
    P.tv_enable=tv;                                
    set_param(model,'StopTime',num2str(t(end)));
    out=sim(model);
    k=sprintf('tv%d',tv);
    R.(k).t=out.tout;
    R.(k).r=out.logsout.get('r').Values.Data;
    R.(k).r_ref=out.logsout.get('r_ref').Values.Data;
    R.(k).ay=out.logsout.get('ay').Values.Data;
    % --- derived signals ---
    delta_t = interp1(t, delta, R.(k).t, 'linear', 'extrap');  % steer on sim grid
    vy      = cumtrapz(R.(k).t, R.(k).ay - P.Vx*R.(k).r);      % lateral velocity
    R.(k).beta = atan2(vy, P.Vx);                              % body slip [rad]

    % --- metrics ---
    err = R.(k).r_ref - R.(k).r;
    R.(k).max_ay    = max(abs(R.(k).ay));
    R.(k).max_r     = max(abs(R.(k).r));
    R.(k).max_err   = max(abs(err));
    R.(k).peak_beta = max(abs(R.(k).beta));

    % understeer onset: first sustained >5% deviation of r below r_ref
    gate = abs(R.(k).r_ref) > 0.1*max(abs(R.(k).r_ref));   % ignore near-zero start
    dev  = abs(err) > 0.05*abs(R.(k).r_ref);
    i0   = find(gate & dev, 1, 'first');
    if isempty(i0)
        R.(k).us_deg = NaN;  R.(k).us_ay = NaN;             % never departed
    else
        R.(k).us_deg = rad2deg(delta_t(i0));               % steer at onset [deg]
        R.(k).us_ay  = R.(k).ay(i0);                       % lateral g at onset
    end
end

fprintf('\n%-24s %10s %10s\n','Metric','TV off','TV on');
fprintf('%-24s %10.3f %10.3f\n','Max ay [m/s^2]',        R.tv0.max_ay,  R.tv1.max_ay);
fprintf('%-24s %10.3f %10.3f\n','Max yaw rate [rad/s]',  R.tv0.max_r,   R.tv1.max_r);
fprintf('%-24s %10.4f %10.4f\n','Max yaw err [rad/s]',   R.tv0.max_err, R.tv1.max_err);
fprintf('%-24s %10.2f %10.2f\n','Understeer onset [deg]',R.tv0.us_deg,  R.tv1.us_deg);
fprintf('%-24s %10.2f %10.2f\n','  ...at ay [m/s^2]',    R.tv0.us_ay,   R.tv1.us_ay);
fprintf('%-24s %10.2f %10.2f\n','Peak |beta| [deg]', ...
    rad2deg(R.tv0.peak_beta), rad2deg(R.tv1.peak_beta));

figure('Name','CP27E TV — off vs on');
subplot(2,1,1); hold on; grid on;
plot(R.tv0.t,R.tv0.r,'b--','DisplayName','r, TV off');
plot(R.tv1.t,R.tv1.r,'b-','DisplayName','r, TV on');
plot(R.tv1.t,R.tv1.r_ref,'k:','DisplayName','r_{ref}');
ylabel('Yaw rate [rad/s]'); legend; title('Yaw-rate tracking');
subplot(2,1,2); hold on; grid on;
plot(R.tv0.t,R.tv0.ay,'r--','DisplayName','a_y, TV off');
plot(R.tv1.t,R.tv1.ay,'r-','DisplayName','a_y, TV on');
xlabel('Time [s]'); ylabel('a_y [m/s^2]'); legend; title('Lateral accel');