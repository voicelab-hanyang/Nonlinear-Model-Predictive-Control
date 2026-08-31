clc
clear 
close all

addpath Input\
%% Initialization
%%=======load vehicle parameter=======%%
load driving_hv1.mat
load driving_hv1_info_space.mat %% TO DO
parameter

%load net_day1
day1 = 1;

allData = cell(1, 3); % 데이터를 저장할 셀 배열
for i = 1:3
    fileName = sprintf('driving_hv%d_info_space', i);
    allData{i} = load(fileName); % 모든 파일 미리 로드
end

% Select epsiode
episode_batchIndices = 1;
data = allData{episode_batchIndices};

% observation
obs.num_s = [2,1];  % the number of state

% action
act.min = -42.0;                                   % minimum action value
act.max = -49.0;                                   % maximum action value
act.interval = -0.5;                               % index action interval
act.num = length(act.min:act.interval:act.max);    % the number of idex action
act.element = act.min:act.interval:act.max;        % element of action

% solver option('Display','final')
options = optimoptions('fmincon',...
'MaxFunctionEvaluations', 1000, 'StepTolerance', 1e-3, 'OptimalityTolerance', 1e-1);
%rng(1); % 난수 생성


%% Create DQN Agent
if day1 == 1
% Define the neural network architecture
params_net.hiddenlayer1 = 40; % the number of neuran in layer1 [-]
params_net.hiddenlayer2 = 30; % the number of neuran in layer2 [-]
params_net.hiddenlayer3 = 20; % the number of neuran in layer3 [-]

net = [
    featureInputLayer(obs.num_s(1), 'Normalization', 'none', 'Name', 'state')  % input-size is 1
    fullyConnectedLayer(params_net.hiddenlayer1, 'Name', 'fc1') % fully connected hidden-layer
    tanhLayer('Name', 'tanh1')  % activation function
    fullyConnectedLayer(params_net.hiddenlayer1, 'Name', 'fc2') % fully connected hidden-layer
    tanhLayer('Name', 'tanh2')  % activation function
    fullyConnectedLayer(act.num, 'Name', 'fc3')]; % fully connected hidden-layer 
net = dlnetwork(net);
end

% Hyperparameters
numEpisodes = 1000;             % episode number [-]
gamma = 0.99;                   % Discout Factor [-]
epsilon = 0.99;                 % Exploration Initial Value [-]
epsilonDecay = 0.990;           % epsilon decay value [-]
minEpsilon = 0.001;             % minimum epsilon [-]
initialLearningRate = 1e-2;     % learning rate [-] 1e-2
learningRateDecay = 0.9;      % Epsilon Decay Factor [-] 0.999

% Experience Replay Memory Initialize
bufferSize = 1e5;                                                % buffer size [-]
replayBuffer = zeros(bufferSize, obs.num_s(1) * 2 + 3);          % [state, action, reward, next_state, done]
bufferCounter = 1;                                               % buffer Counte number

% Initialize optimizer variables
gradThreshold = 1.0;            % gradient climping ctrical
beta1 = 0.9;
beta2 = 0.999;
epsilonOpt = 1e-8;              % epsilon using of optimization
movingAvg = [];
movingAvgSq = [];

targetNet = updateTargetNet(net);  % update target network     
%plot(env)

% Initialize history
res.rewards = [];

% monitoring learning progress
monitor = trainingProgressMonitor( ...
    Metrics="EpisodeReward", ...
    Info=["Episode","SOC","step_i","epsilon"], ...
    XLabel="EpisodeNumber");
pause(5)
%% DQN Learning
for episode = 1:numEpisodes
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
res.reward = 0;

val.theta = 0;
val.s = s_interp(1:opti.Ds:length(v_interp));
val.vel = v_interp(1:opti.Ds:length(v_interp));
val.segment_norm = (i*opti.Ds) / 10000;

% val.accel2 = diff(val.vel);
for i = 2:1:length(val.vel)
val.accel(i-1) = (val.vel(i)^2-val.vel(i-1)^2)/(2*opti.Ds);
end
val.accel(end+1) = 0;

state  = [init_SOC;val.segment_norm ];                                  % reset initial state [SOC, distance]
totalReward = 0;                                                              % reset initial totla reward [-]
currentLearningRate = initialLearningRate * learningRateDecay^(episode - 1);  % decay learning rate [-]

 
    for i = 1:1:length(val.s)-(opti.Np+1)
        % epsilon-greedy (select action)
        if rand < epsilon
            actionIdx = randi([1, act.num]);
        else
            dlState = dlarray(state, 'CB'); % 'CB' 차원 태그 사용
            qValues = predict(net, dlState);
            [~, actionIdx] = max(extractdata(qValues));
        end
        
        % 행동을 올바른 값으로 매핑
        action = act.element(actionIdx);  % 행동을 올바른 값으로 매핑
        
        %% implement MPC (environment)
   
   % define initial SOC(k)
    xk = init_SOC;
    
    % define velocity & accel. reference
    val.vref = val.vel(i:1:i+(opti.Np));
    val.aref = val.accel(i:1:i+(opti.Np));

    % if val.accel(i) > 0
    [u_seq,fval,exitflag] = NMPC(val,params,opti,xk,action);
    % else
    %     u_seq = 0;
    % end

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
    distance = i*opti.Ds
   

     val.segment_norm = (i*opti.Ds) / 10000;

        % nest state(s(t+1))     
        nextState = [res.SOC(i+1);val.segment_norm];

        % reward (R(t))
        reward = -10*abs(res.SOC(i)-opti.SOC_f) - 0.1*res.m_eq_dot(i+1);
        % reward = res.m_dot(i+1) + (res.SOC(i)-res.SOC(i+1)) * params.e_cap * 3600 * params.V / (params.heat_val/3600);
        % reward = res.m_dot(i+1) + (res.SOC(i+1) - opti.SOC_f)*44;

        % end statues
        %if params.state_min >= valsk.SOC || valsk.SOC >= params.state_max
            % || (params.SOC_target-params.slack) >= valsk.SOC||...
            %    valsk.SOC >= (params.SOC_target+params.slack)
        %isDone = 1;
        %else
        isDone = 0;    
        %end
        totalReward = totalReward + reward;


        %% 경험 저장
        experience = [state', actionIdx, reward, nextState', isDone];
        if bufferCounter <= bufferSize
            replayBuffer(bufferCounter, :) = experience;
        else
            idx = mod(bufferCounter - 1, bufferSize) + 1;
            replayBuffer(idx, :) = experience;
        end
        bufferCounter = bufferCounter + 1;
        
        state = nextState;
        
        if isDone
            break;
        end
        
        % 경험 재생 메모리에서 미니배치 샘플링
        if bufferCounter > 256
            batchIndices = randi(min(bufferCounter-1, bufferSize), [256, 1]);
            batch = replayBuffer(batchIndices, :);
            
            % 배치에서 데이터 추출
            states = batch(:, 1:obs.num_s(1));
            actions = batch(:, obs.num_s(1) + 1);
            rewards = batch(:, obs.num_s(1) + 2);
            nextStates = batch(:, obs.num_s(1) + 3:obs.num_s(1) * 2 + 2);
            dones = batch(:, end);

            % 타깃 Q 값 계산
            dlNextStates = dlarray(nextStates', 'CB'); % 'CB' 차원 태그 사용
            nextQValues = predict(targetNet, dlNextStates);
            maxNextQValues = max(extractdata(nextQValues), [], 1)';
            targetQValues = rewards + gamma * maxNextQValues .* (1 - dones);
            
            % Q 값 예측 및 손실 계산
            dlStates = dlarray(states', 'CB'); % 'CB' 차원 태그 사용
            qValues = predict(net, dlStates);
            indices = sub2ind([act.num,size(qValues,2)], actions, (1:length(actions))');  % 수정된 인덱싱
            qValues = qValues(indices);
                        
            % 경사 하강법으로 네트워크 업데이트
            [gradients, net] = dlfeval(@modelGradients, net, dlStates, targetQValues, indices);
            gradients = dlupdate(@(g) min(max(g, -gradThreshold), gradThreshold), gradients);  % 그래디언트 클리핑
            [net, movingAvg, movingAvgSq] = adamupdate(net, gradients, movingAvg, movingAvgSq, bufferCounter, currentLearningRate, beta1, beta2, epsilonOpt);  
        end
    end
    save('net.mat','net');

    % epsilon 감소
    epsilon = max(minEpsilon, epsilon * epsilonDecay);

    
    % 타깃 네트워크 업데이트
    if mod(episode, 4) == 0  % 타깃 네트워크 업데이트 주기 증가
        targetNet = updateTargetNet(net);
    end
    pause(3)
    % 학습 진행 모니터링 업데이트
    recordMetrics(monitor,episode, ...
    EpisodeReward=totalReward);
    updateInfo(monitor,Episode=episode,SOC=res.SOC(end),step_i=i,epsilon=epsilon);
    monitor.Progress = 100*episode/numEpisodes;

    % 학습 진행 상황 출력
    fprintf('Episode %d, Total Reward: %.2f, Epsilon: %.2f\n', episode, totalReward, epsilon);
    res.rewards = [res.rewards; totalReward];
end

    save('net.mat','net');

%% 모형 경사 계산 함수
function [gradients, dlnet] = modelGradients(dlnet, dlStates, targetQValues, indices)
    qValues = predict(dlnet, dlStates);
    qValues = qValues(indices);
    loss = mean((targetQValues - qValues).^2);
    gradients = dlgradient(loss, dlnet.Learnables);
end

% 네트워크 복사 함수
function targetNet = updateTargetNet(dlnet)
    targetNet = dlnet;
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
                         % value function

% computation: first step 
[x_seq(:,2), m_dot_seq(:,2)] = Euler(x_seq(:,1), u_seq(:,1), i_seq(:,1),  params, val, opti);

% ignore current state, x_seq(:,1), since it is independent from control
JN = JN + m_dot_seq(:,2) + (x_seq(:,2)-opti.SOC_f)*action;
        
% computation: from step 2 to step Nc
for ii = 2 : opti.Nc
    % propagate state forward
    [x_seq(:,ii+1), m_dot_seq(:,ii+1)] = Euler(x_seq(:,ii), u_seq(:,ii), i_seq(:,ii),  params, val, opti);
    % calculate cost
    JN = JN + m_dot_seq(:,ii+1) + (x_seq(:,ii+1) - opti.SOC_f)*action;
 end

% computation: from step Nc+1 to step Np
for jj = opti.Nc+1 : 1 : opti.Np
    % propagate state forward
    [x_seq(:,jj+1),m_dot_seq(:,jj+1)] = Euler(x_seq(:,jj), u_seq(:,end), i_seq(:,jj),  params, val, opti);
    % calculate cost
    JN = JN + m_dot_seq(:,jj+1) + (x_seq(:,jj+1) - opti.SOC_f)*action;
    
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