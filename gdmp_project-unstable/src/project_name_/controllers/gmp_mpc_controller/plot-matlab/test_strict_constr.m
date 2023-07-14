clc;
% close all;
clear;

import_gmp_lib();
import_io_lib();
addpath('utils/');


%% -------- load GMP model --------
gmp = GMP();
gmp_.read(gmp, '../config/model.bin');

n_dof = gmp.numOfDoFs();

%% -------- Limits ---------
pos_lim = [
    [ -0.2 , 0.65];
    [ -0.7 , 0.3];
    [ 0.05 , 0.6];
  ];

vel_lim = repmat([ -0.4 , 0.4 ], n_dof, 1);

accel_lim = repmat([ -0.8 , 0.8 ], n_dof, 1);

% pos_lim = pos_lim + repmat([-1 1], n_dof, 1); % to augment the limits

%% --------- Optimization objective ----------
opt_pos = 0;
opt_vel = (1 - opt_pos);

%% -------- Initial/Final states ---------

ygd = [0.26, -0.12, 0.32]';   % demo target
yg = [ 0.55, 0.0817, 0.46 ]'; % execution target
tau = 3.5; % execution time duration

%% to change online the target pose at t = t_g from 'yg0' to 'yg'
t_g = inf; %0.5*tau;
yg0 = yg;

y0 = [-0.0956; -0.443; 0.196];

gmp.setY0(y0);
gmp.setGoal(yg0);

%% -------- via-points ----------
vp_config.t_ahead = [];
vp_config.via_points = {};

%% ------- QP solver ---------
qp_solver_type = 0; % matlab-quadprog:0 , OSQP:1, Goldfarb-Idnani: 2

%% ------ Methods to use -----------
demo     = 1; % plot demo
dmp_prop = 1; % original DMP
dmp_rot  = 0; % DMP with rotational-scaling
dmp_opt  = 0; % optimal GMP
dmp_mpc  = 1; % MPC GMP
dmp_rf   = 0; % original DMP with rep-forces
dmp_qp   = 0; % DMP-QP 2015
traj_opt = 0; % LQR with reference from DMP 
traj_mpc = 0; % MPC with reference from DMP

%% =============  Run  ==================

run_simulations();

%% ======= Plot results =========

plot_results(data, y0, yg, ygd, tau, pos_lim, vel_lim, accel_lim);
