%% Vehicle Parameter

%%========Autonomie initialization=======%%
ess.plant.init.soc_init = 0.6;                   % simulation time [s]
params.e_cap = 5.9430;
%%========Define Battery=========%%
params.V_index1 = ess.plant.init.voc.idx2_soc(1,:);         % SOC index for battery voltage [-]
params.V_map = ess.plant.init.voc.map(2,:);                 % battery volage map [V]
params.cell_series = ess.plant.init.num_cell_series;        % cell series number [-]
params.cell_parallel = ess.plant.init.num_module_parallel;  % cell parallel number [-]
params.soc_init = ess.plant.init.soc_init;                  % initial SOC [-]   
params.e_cap = ess.plant.init.cap_max.map(1);              % battery capacity [Ah]
params.rdis_index1 = ess.plant.init.rint_dis.idx2_soc;      % SOC index for discharging resistance [ohm]
params.rdis = ess.plant.init.rint_dis.map(2,:);             % discharging resistance [ohm]
params.rchg_index1 = ess.plant.init.rint_chg.idx2_soc;      % SOC index for charging resistance [ohm] 
params.rchg = ess.plant.init.rint_chg.map(2,:);             % charging resistance [ohm]
params.V = 360;

%%=======Define Motor=========%
params.Pm_idx1_spd = mot.plant.calc.pwr_elec.idx1_spd;  % speed data for motor map (motor_index_x-axis)
params.Pm_idx2_trq = mot.plant.calc.pwr_elec.idx2_trq;  % torque data for motormap (motor_index_y-axis)
params.Pm_map_data = mot.plant.calc.pwr_elec.map;       % motor power map (fuel_index_z-axis)
params.motor_spd_range = mot.plant.init.trq_max.idx1_spd;
params.T_m_max_map = mot.plant.init.trq_max.map;
params.T_m_min_map = mot.plant.init.trq_min.map;
%%=======Define engine=========%
params.fuel_idx1_spd = eng.plant.init.fuel_hot.idx1_spd;  % speed data for fuel consumtion map (fuel_index_x-axis)
params.fuel_idx2_trq = eng.plant.init.fuel_hot.idx2_trq;  % torque data for fuel consumtion map (fuel_index_y-axis)
params.fuel_map_data = eng.plant.init.fuel_hot.map;       % fuel rate map (fuel_index_z-axis)
params.engine_spd_range = eng.plant.init.trq_max_hot.idx1_spd;
params.T_e_max_map = eng.plant.init.trq_max_hot.map ;
params.T_e_idle = eng.plant.init.spd_idle;
params.heat_val = eng.plant.init.fuel_heating_val;


%%=======Define chassis=======%
params.A = chas.plant.init.area_drag;
params.f = 0.01;
params.m = veh.plant.init.mass.total;
params.rw = whl.plant.init.radius;


%%======Define env=========%
params.rho = env.init.dens_air;
params.g = env.init.gravity;

%%======Define final drive=====%
params.i0 = fd.plant.init.ratio;

%%======Define gear=====%
params.i_num = gb.plant.init.ratio.idx1_gear;
params.i_g = 1.5;

%%======System parameter====%%
params.nx = 1;           % number of states
params.ny = 1;           % number of outputs
params.nu = 1;           % number of control inputs

%%====fule fitting====%%
% generate 2d grid map (fuel consumption)

params.a0 = 3.371446e-10;  % w_e^2 계수
params.a1 = 5.677485e-08;  % w_e*T_e 계수  
params.a2 = -1.653664e-09; % T_e^2 계수
params.a3 = 8.276302e-07;  % w_e 계수
params.a4 = 7.037429e-07;  % T_e 계수
params.a5 = -7.116979e-06; % 상수항


%%====Mot power fitting====%%

params.b0 = 3.929978e-02;                 % motor poser coefficient b0;
params.b1 =1.019568e+00;                 % motor poser coefficient b1;
params.b2 = 1.998951e-01;                 % motor poser coefficient b2;
params.b3 = 2.230537e-15;                 % motor poser coefficient b3;
params.b4 = 7.772227e-14;                 % motor poser coefficient b4;
params.b5 = -1.833476e+03;                 % motor poser coefficient b5;

%% Optimization parameter
%%=====Optimization====%%
opti.Nc = 8;           % Control horizon [s]
opti.Np = 10;          % Prediction horizon [s] 
opti.ts = 10;        % Sampling distance [m]

opti.Q1 = 500;         % Weight factor for state 1 [-]
opti.Q2 = 100;         % Weight factor for state 2 [-]
opti.R = 1;            % Weight factor for increasing control input [-]

opti.IterMAX = 500;  % Maximum Iterations [-]

% Algorithm
opti.algrtm = 'sqp';

%Sample distance
opti.Ds = 30;

% target soc
opti.SOC_f = 0.6;


