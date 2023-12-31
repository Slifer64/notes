clc;
close all;
clear;

%% =============  includes...  =============
import_gmp_lib();
addpath('utils/')

%% =============  Load demo data  =============                           
load('data/demo_data2.mat', 'Timed', 'Pd_data');

n_dof = size(Pd_data, 1);
% calc vel/accel numerically for plotting them later
dPd_data = [diff(Pd_data, 1, 2)./diff(Timed) zeros(n_dof,1)];
ddPd_data = [diff(dPd_data, 1, 2)./diff(Timed) zeros(n_dof,1)];

%% ============= Train DMP  =============
gmp = trainDMP(Timed, Pd_data);


%% =============  DMP simulation  =============
disp('Simulation...');
t_start = tic;

%% Initial/Final values
y0d = Pd_data(:,1);   % Initial demo position
gd = Pd_data(:,end); % Target demo position
y0 = y0d; % set initial position for execution (for simplicity lets leave it the same as the demo)
g = gd + 0.4;  % set target position for execution
Tf = Timed(end); % set the time duration of the executed motion

dt = 0.005; % time step for numerical integration

%% Simulate DMP
Ts = 1.0; % target steps every Ts sec
get_target_fun = @(t) gd + (g-gd)*min([Ts*ceil(t/Ts), Tf])/Tf;
% get_target_fun = @(t) g;
[Time, P_data, dP_data, ddP_data, Pg_data] = simulateModel(gmp, y0, get_target_fun, Tf, dt);
[Time2, P2_data, dP2_data, ddP2_data, Pg2_data] = simulateDMP(gmp, y0, get_target_fun, Tf, dt);

toc(t_start)

%% Plot results
plot_DMP_comparison_1DoF();


%% ============================================================
%% ============================================================
