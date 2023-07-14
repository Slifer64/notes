clc;
close all;
clear;

%% =============  includes...  =============
import_gmp_lib();
addpath('utils/')

%% =============  Load demo data  =============                           
load('data/demo_data1.mat', 'Timed', 'Pd_data');

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
g = -gd;  % set target position for execution
Tf = Timed(end); % set the time duration of the executed motion

dt = 0.005; % time step for numerical integration

%% Simulate DMP
get_target_fun = @(t) g;
[Time, P_data, dP_data, ddP_data] = simulateModel(gmp, y0, get_target_fun, Tf, dt);
[Time2, P2_data, dP2_data, ddP2_data] = simulateDMP(gmp, y0, get_target_fun, Tf, dt);

% Timed = Time;
% Pd_data = P_data;
% save('demo_same_start_goal.mat', 'Timed', 'Pd_data');
% return

toc(t_start)

% Pd0_data = interp1(Timed, Pd_data, Time, 'linear', g);
% err = sum(vecnorm(Pd0_data - P_data, 2, 1))
% err2 = sum(vecnorm(Pd0_data - P2_data, 2, 1))

%% Plot results
plot_DMP_comparison_1DoF();



%% ============================================================
%% ============================================================
