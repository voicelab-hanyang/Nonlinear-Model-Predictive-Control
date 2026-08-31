clc;clear;

addpath Input\

%%=======load speed profile=======%%
load driving_hv1.mat
load driving_udds_info_space.mat    
%%=======load vehicle parameter=======%%
parameter
%%=======load network=======%%
load net
act.min = -42.0;                                   % minimum action value
act.max = -49.0;                                   % maximum action value
act.interval = -0.5;                               % index action interval
act.num = length(act.min:act.interval:act.max);    % the number of idex action
act.element = act.min:act.interval:act.max;        % element of action
%%=======road grade=======%%
val.theta = 0;
val.s = s_interp(1:opti.Ds:length(v_interp));
val.vel = v_interp(1:opti.Ds:length(v_interp));
% val.accel2 = diff(val.vel);
for i = 2:1:length(val.vel)
val.accel(i-1) = (val.vel(i)^2-val.vel(i-1)^2)/(2*opti.Ds);
end
val.accel(end+1) = 0;
%%======remaiend distance====%%
d_r = val.s(end)-1000;


%%======Initial value=====%%
init_SOC = ess.plant.init.soc_init;
res.SOC = init_SOC; 
res.m_dot = 0;
res.m_cum = 0;
res.m_eq = 0;
res.m_eq_dot = 0;
res.T_m = 0;
res.T_d = 0;
res.T_e = 0;
res.w_d = 0;
res.w_m = 0;
res.w_e = 0;
distance = 0;
res.action = 0;

%% Calculate NMPC

for i = 1:1:length(val.s)-(opti.Np+1)
    segment_norm = (i*opti.Ds) / 10000;
    % define initial SOC(k)
    xk = init_SOC;
    
    if d_r > i*opti.Ds
    state  = [xk;segment_norm];           % define state
    dlState = dlarray(state, 'CB');               % 'CB' 차원 태그 사용
    qValues = predict(net, dlState);
    [~, actionIdx] = max(extractdata(qValues));
    action = act.element(actionIdx);              % 행동을 올바른 값으로 매핑

    else
    action = act.max;
    end
    %%%%%%%%%%%%%%% TO DO %%%%%%%
    action = -100;
    %%%%%%%%%%%%%%%    %%%%%%%%%%%%
    % define velocity & accel. reference
    val.vref = val.vel(i:1:i+(opti.Np));
    val.aref = val.accel(i:1:i+(opti.Np));


    [u_seq,fval,exitflag] = NMPC(val,params,opti,xk,action);


    % save history
    res.exitflag(i) = exitflag;
    ik = [val.vel(i),val.accel(i)];
    [res.SOC(i+1), ~] = Euler(xk, u_seq(1), ik, params, val, opti);
    init_SOC = res.SOC(i+1);
    res.u(i) = u_seq(1);
    [res.T_d(i+1), res.w_d(i+1), res.T_m(i+1), res.w_m(i+1), res.T_e(i+1), res.w_e(i+1), res.m_dot(i+1)] = EulerTorque(xk, u_seq(1), ik, params, val, opti);
    res.m_eq_dot(i+1) = res.m_dot(i+1) + (res.SOC(i)-res.SOC(i+1)) * params.e_cap * 3600 * params.V / (params.heat_val/3600);
    res.m_eq(i+1) = res.m_eq(i) + res.m_eq_dot(i+1);
    res.m_cum(i+1) = res.m_cum(i) + res.m_dot(i+1);
    res.action(i+1) = action;
    distance = i*opti.Ds
   
end


%% NMPC FUNCTION
function [u_seq,fval,exitflag] = NMPC(val,params,opti,xk,action)

% Define variable
varsk.U_max = kron(ones(opti.Nc,1),1);          % umax (Nc X 1)
varsk.theta = val.theta;                        % theta [rad]
varsk.SOC = xk(1);                              % SOC [-]
i_seq(1,:) = val.vref;                 % velocity reference [m/s]
i_seq(2,:) = val.aref;                 % accel reference [m/s]


% fmincon option
nmpc_options = optimoptions('fmincon',...
                           'Display','off',...
                           'Algorithm', opti.algrtm,... % 
                           'SpecifyConstraintGradient',false,...
                           'UseParallel',false,...
                           'MaxIterations',opti.IterMAX);


[u_seq,fval,exitflag] = fmincon(@(u_seq)costfunc(u_seq, xk,  params, val, opti, action),...
                    ones(1,opti.Nc).*0.5,...
                    [],...
                    [],...
                    [],...
                    [],...
                    varsk.U_max*0,...
                    varsk.U_max,...
                    @(u_seq)getNonlinearConstraint(u_seq, xk, params, val, opti, i_seq),...
                    nmpc_options);

end


%% Cost function
function JN = costfunc(u_seq, xk, params, val, opti, action)

% initialization
x_seq = zeros(params.nx, opti.Np+1);   % state sequence
m_dot_seq = zeros(1, opti.Np+1);       % fuel rate
x_seq(:,1) = xk;                       % cureent state (xt) 
i_seq(1,:) = val.vref;                 % velocity reference [m/s]
i_seq(2,:) = val.aref;                 % accel reference [m/s]
JN = 0;                                % initial cost
V_f = action;                           % value function
%42~49
% computation: first step 
[x_seq(:,2), m_dot_seq(:,2)] = Euler(x_seq(:,1), u_seq(:,1), i_seq(:,1),  params, val, opti);

% ignore current state, x_seq(:,1), since it is independent from control
JN = JN + m_dot_seq(:,2) + (x_seq(:,2)-opti.SOC_f)*V_f;
        
% computation: from step 2 to step Nc
for ii = 2 : opti.Nc
    % propagate state forward
    [x_seq(:,ii+1), m_dot_seq(:,ii+1)] = Euler(x_seq(:,ii), u_seq(:,ii), i_seq(:,ii),  params, val, opti);
    % calculate cost
    JN = JN + m_dot_seq(:,ii+1) + (x_seq(:,ii+1) - opti.SOC_f)*V_f;
 end

% computation: from step Nc+1 to step Np
for jj = opti.Nc+1 : 1 : opti.Np
    % propagate state forward
    [x_seq(:,jj+1),m_dot_seq(:,jj+1)] = Euler(x_seq(:,jj), u_seq(:,end), i_seq(:,jj),  params, val, opti);
    % calculate cost
    JN = JN + m_dot_seq(:,jj+1) + (x_seq(:,jj+1) - opti.SOC_f)*V_f;
    
end

end

%% RK4
function [xkp1, m_dot] = Euler(xk, uk, ik, params, val, opti)

vk = ik(1); % speed(k) [m/s]
ak = ik(2); % accel.(k) [m/s^2]

% time interval (s/v)
if vk < 1
    time_interval = 0;
else
time_interval = opti.Ds/(vk+0.01);
end

% Calculate force
F_a = 0.5*params.rho*params.A*vk.^2;                % Aero resistance [N]
F_r = cos(val.theta) * params.f*params.m*params.g;  % Rolling resistance [N]
F_g = sin(val.theta) * params.g * params.m;         % Slope resistance [N]
F_t = params.m * ak;                                % Traction force [N]             

% Demanded torque 
F_d = F_a + F_r + F_g + F_t;                        % Demanded torque [N]

% Wheel Torque & Speed (speed to wheel)
w_w = vk / params.rw;                   % wheel speed [rad]
T_w = F_d * params.rw;                  % wheel torque [Nm]

% final drive (final to powertrain)
w_f = w_w*params.i0;                    % final to powertrain speed [rad]
T_f = T_w/params.i0;                    % final to powertrain torque [Nm]

% demanded drive (final to demanded)
T_d = T_f ./ params.i_g;                % demanded torqe [Nm]
w_d = w_f .* params.i_g;                % demanded speed [rad]

if uk == 1
    w_m = 0;
else
w_m = w_d;                              % motor speed [rad]
end

T_m = T_d*(1-uk);                       % motor torque [Nm]

%P_m = interp2(params.Pm_idx1_spd, params.Pm_idx2_trq, params.Pm_map_data', w_m, T_m, 'linear',0);
P_m =  params.b0*w_m^2 + params.b1*w_m*T_m + params.b2*T_m^2 + params.b3*w_m + params.b4*T_m + params.b5;

I_m= P_m/ params.V;
e_cap = -I_m * time_interval / 3600 + params.e_cap*xk(1);
xkp1 = e_cap/params.e_cap;

w_e = w_d;
T_e = T_d*(uk);

% m_dot = interp2(params.fuel_idx1_spd, params.fuel_idx2_trq, params.fuel_map_data', w_e, T_e, 'linear',0)*time_interval*1000;
m_dot= (params.a0*w_e^2 + params.a1*w_e*T_e + params.a2*T_e^2 + params.a3*w_e + params.a4*T_e + params.a5)*time_interval*1000;

end

%% Constraint function
function [c, c_eq] = getNonlinearConstraint(u_seq, xk, params, val, opti, i_seq)

% X_min <= X <= X_max 
% X = nx*2 * Np
% -------------------------------------------------------------------
% reshape u_seq from column vector (nu*Nc, 1) to matrix (nu, Nc)
u_seq = reshape(u_seq, params.nu, opti.Nc);

% initialization
x_seq = zeros(params.nx, opti.Np+1); x_seq(:,1) = xk;  % state sequence


% computation: from step 1 to step Nc
for j = 1 : opti.Nc
    x_seq(:,j+1) = Euler(x_seq(:,j), u_seq(:,j), i_seq(:,j), params, val, opti);
end
% computation: from step Nc+1 to step Np
for jj = opti.Nc+1 : 1 : opti.Np
    % propagate state forward
    x_seq(:,jj+1) = Euler(x_seq(:,jj), u_seq(:,end), i_seq(:,jj), params, val, opti);
end
x_min = ones(params.nx, opti.Np+1).*0.5;
x_max = ones(params.nx, opti.Np+1).*0.7;
c = [x_min - x_seq;      % x_min - x_seq <= 0  →  x_seq >= x_min
     x_seq - x_max];     % x_seq - x_max <= 0  →  x_seq <= x_max

% reshape nonlinear constraint to match fmincon format
c = reshape(c, [], 1);

% No nonlinear equality constraint
c_eq = [];

end

%%
function [T_d, w_d, T_m, w_m, T_e, w_e, m_dot] = EulerTorque(xk, uk, ik, params, val, opti)

vk = ik(1); % speed(k) [m/s]
ak = ik(2); % accel.(k) [m/s^2]

% time interval (s/v)
if vk < 1
    time_interval = 0;
else
time_interval = opti.Ds/(vk+0.01);
end

% Calculate force
F_a = 0.5*params.rho*params.A*vk.^2;                % Aero resistance [N]
F_r = cos(val.theta) * params.f*params.m*params.g;  % Rolling resistance [N]
F_g = sin(val.theta) * params.g * params.m;         % Slope resistance [N]
F_t = params.m * ak;                                % Traction force [N]             

% Demanded torque 
F_d = F_a + F_r + F_g + F_t;                        % Demanded torque [N]

% Wheel Torque & Speed (speed to wheel)
w_w = vk / params.rw;                   % wheel speed [rad]
T_w = F_d * params.rw;                  % wheel torque [Nm]

% final drive (final to powertrain)
w_f = w_w*params.i0;                    % final to powertrain speed [rad]
T_f = T_w/params.i0;                    % final to powertrain torque [Nm]

% demanded drive (final to demanded)
T_d = T_f ./ params.i_g;                % demanded torqe [Nm]
w_d = w_f .* params.i_g;                % demanded speed [rad]

if uk == 1
    w_m = 0;
else
w_m = w_d;                              % motor speed [rad]
end
T_m = T_d*(1-uk);                       % motor torque [Nm]

%P_m = interp2(params.Pm_idx1_spd, params.Pm_idx2_trq, params.Pm_map_data', w_m, T_m, 'linear',0);
P_m =  params.b0*w_m^2 + params.b1*w_m*T_m + params.b2*T_m^2 + params.b3*w_m + params.b4*T_m + params.b5;

I_m= P_m/ params.V;
e_cap = -I_m * time_interval / 3600 + params.e_cap*xk(1);
xkp1 = e_cap/params.e_cap;

w_e = w_d;
T_e = T_d*(uk);
if T_e<0
    T_e = 0;
end
% m_dot = interp2(params.fuel_idx1_spd, params.fuel_idx2_trq, params.fuel_map_data', w_e, T_e, 'linear',0)*time_interval*1000;
m_dot = (params.a0*w_e^2 + params.a1*w_e*T_e + params.a2*T_e^2 + params.a3*w_e + params.a4*T_e + params.a5)*time_interval*1000;

end