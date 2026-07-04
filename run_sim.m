params;

mode = 'ramp';          % 'step' | 'ramp' | 'steplimit' | 'skidpad'
dt = 1e-3;
switch mode
    case 'step'                                    % gentle step (verification)
        t=(0:dt:5)'; delta=zeros(size(t)); delta(t>=1)=0.05;
    case 'ramp'                                    % sweep up to the limit
        t=(0:dt:15)'; delta=min(t,5)/5*0.40;
    case 'steplimit'                               % step steer near the limit
        t=(0:dt:5)'; delta=zeros(size(t)); delta(t>=1)=0.35;
    case 'skidpad'                                 % steady constant-radius corner
        t=(0:dt:6)'; delta=0.35*ones(size(t));
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
    ds = out.get('logsout');
    k=sprintf('tv%d',tv);
    R.(k).t=out.tout;
    R.(k).r=out.logsout.get('r').Values.Data;
    R.(k).r_ref=out.logsout.get('r_ref').Values.Data;
    R.(k).ay   = out.logsout.get('ay').Values.Data;
    R.(k).Vx   = out.logsout.get('Vx').Values.Data;      
    R.(k).beta = out.logsout.get('beta').Values.Data; 

    % --- derived signals ---
    delta_t = interp1(t, delta, R.(k).t, 'linear', 'extrap');  % steer on sim grid
    vy = R.(k).Vx .* tan(R.(k).beta);                           % lateral velocity

    % --- trajectory reconstruction (world frame, variable speed) ---
    psi = cumtrapz(R.(k).t, R.(k).r);                    % heading [rad]
    Xd  = R.(k).Vx.*cos(psi) - vy.*sin(psi);
    Yd  = R.(k).Vx.*sin(psi) + vy.*cos(psi);
    R.(k).X = cumtrapz(R.(k).t, Xd);
    R.(k).Y = cumtrapz(R.(k).t, Yd);

    % --- metrics ---
    err = R.(k).r_ref - R.(k).r;
    ss  = R.(k).t > 0.6*t(end);
    R.(k).max_ay=max(abs(R.(k).ay)); R.(k).peak_beta=max(abs(R.(k).beta));
    R.(k).ss_err = mean(abs(err(ss)));
    R.(k).r_ss   = mean(R.(k).r(ss));
    R.(k).us_pct = 100*(1 - R.(k).r_ss/mean(R.(k).r_ref(ss)));
    R.(k).Vexit = R.(k).Vx(end);

    % per-wheel signals
    R.(k).T_FL = ds.get('T_FL').Values.Data;
    R.(k).T_FR = ds.get('T_FR').Values.Data;
    R.(k).T_RL = ds.get('T_RL').Values.Data;
    R.(k).T_RR = ds.get('T_RR').Values.Data;
    R.(k).Fz   = squeeze(ds.get('Fz').Values.Data)';   % N x 4 [FL FR RL RR]


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

fprintf('\n%-22s %10s %10s\n','Metric','TV off','TV on');
fprintf('%-22s %10.3f %10.3f\n','Max ay [m/s^2]',   R.tv0.max_ay, R.tv1.max_ay);
fprintf('%-22s %10.3f %10.3f\n','SS yaw rate [rad/s]',R.tv0.r_ss, R.tv1.r_ss);
fprintf('%-22s %10.4f %10.4f\n','SS yaw err [rad/s]',R.tv0.ss_err, R.tv1.ss_err);
fprintf('%-22s %10.2f %10.2f\n','SS understeer [%]',R.tv0.us_pct, R.tv1.us_pct);
fprintf('%-22s %10.2f %10.2f\n','Peak |beta| [deg]',rad2deg(R.tv0.peak_beta), rad2deg(R.tv1.peak_beta));
fprintf('%-24s %10.2f %10.2f\n','Exit speed [m/s]', R.tv0.Vexit, R.tv1.Vexit);

%% ---- Top-down trajectory, colored by |yaw rate| ----
cmap = [linspace(0,1,256)' linspace(1,0,256)' zeros(256,1)];  % green -> red
rmax = max([abs(R.tv0.r); abs(R.tv1.r)]);

figure('Name','Trajectory — color = |yaw rate|');
tl = {'TV off','TV on'};  keys = {'tv0','tv1'};
for i = 1:2
    subplot(1,2,i);
    k = keys{i};
    scatter(R.(k).X, R.(k).Y, 8, abs(R.(k).r), 'filled');
    colormap(cmap); clim([0 rmax]); colorbar;
    axis equal; grid on;
    xlabel('X [m]'); ylabel('Y [m]'); title(tl{i});
end

%% ---- Reference (ideal) circle: perfect tracking of r_ref, no body slip ----
psi_ref = cumtrapz(R.tv0.t, R.tv0.r_ref);
Xref = cumtrapz(R.tv0.t, R.tv0.Vx.*cos(psi_ref));
Yref = cumtrapz(R.tv0.t, R.tv0.Vx.*sin(psi_ref));

%% ---- Trajectory overlay: reference (gray) vs TV off (white->red) vs TV on (green->blue) ----
figure('Name','Trajectory overlay — reference vs TV on vs TV off');
hold on; grid on; axis equal;
plot(Xref, Yref, 'Color',[0.5 0.5 0.5], 'LineWidth',1.5, 'DisplayName','Reference');
plotGradientLine(R.tv0.X, R.tv0.Y, [1 1 1], [1 0 0], 0.6, 2.5, 'TV off');
plotGradientLine(R.tv1.X, R.tv1.Y, [0 1 0], [0 0 1], 0.6, 2.5, 'TV on');

xlabel('X [m]'); ylabel('Y [m]');
title('Trajectory overlay: reference (gray), TV off (white\rightarrowred), TV on (green\rightarrowblue)');
legend show;

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

function plotGradientLine(x, y, c1, c2, alphaVal, lw, name)
n = numel(x);
x = x(:)'; y = y(:)';
frac = linspace(0,1,n)';
col = (1-frac).*c1 + frac.*c2;      % n-by-3 RGB gradient
cdata = zeros(2,n,3);
cdata(1,:,:) = col; cdata(2,:,:) = col;
surface('XData',[x;x], 'YData',[y;y], 'ZData',zeros(2,n), ...
        'CData',cdata, 'FaceColor','none', 'EdgeColor','interp', ...
        'EdgeAlpha',alphaVal, 'LineWidth',lw, 'DisplayName',name);
end

%% ---- Per-wheel torques and vertical loads (TV on) ----
figure('Name','Per-wheel torque & load — TV on');
subplot(2,1,1); hold on; grid on;
plot(R.tv1.t, R.tv1.T_FL, 'DisplayName','T_{FL}');
plot(R.tv1.t, R.tv1.T_FR, 'DisplayName','T_{FR}');
plot(R.tv1.t, R.tv1.T_RL, 'DisplayName','T_{RL}');
plot(R.tv1.t, R.tv1.T_RR, 'DisplayName','T_{RR}');
ylabel('Motor torque [Nm]'); legend; title('Wheel torques');
subplot(2,1,2); hold on; grid on;
plot(R.tv1.t, R.tv1.Fz(:,1), 'DisplayName','Fz_{FL}');
plot(R.tv1.t, R.tv1.Fz(:,2), 'DisplayName','Fz_{FR}');
plot(R.tv1.t, R.tv1.Fz(:,3), 'DisplayName','Fz_{RL}');
plot(R.tv1.t, R.tv1.Fz(:,4), 'DisplayName','Fz_{RR}');
xlabel('Time [s]'); ylabel('Vertical load [N]'); legend; title('Wheel loads');