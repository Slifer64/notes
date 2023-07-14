clc;
close all;
clear;

import_gmp_lib();
import_io_lib();

train_data_file = '../data/train_data.bin';
gmp_out_model_file = '../config/model.bin';


%% ====== Load train data ======
fid = FileIO(train_data_file, FileIO.in);
% data = fid.readAll();
Timed = fid.read('Timed');
Pd_data = fid.read('Pd_data');
joint_pos_data = fid.read('joint_pos_data');

j_start = joint_pos_data(:,1)'
Pgd = Pd_data(:,end)'
tau = Timed(end)

%% ====== Train model ======
n_dof = size(Pd_data, 1);
N_kernels = 30;
kernels_std_scaling = 1.5;

xd_data = Timed/Timed(end);

gmp = GMP(n_dof, N_kernels, kernels_std_scaling);
train_err = gmp.train('LS', xd_data, Pd_data)

%% ====== Generate trajectory from model ======

kt = 1.5;
P0 = Pd_data(:,1);
Pg = Pd_data(:,end) + [0.1; 0.1; 0.2];

Time = Timed / kt;
tau = Time(end);

n_data = length(xd_data);
x_dot = 1/tau;
x_ddot = 0;

gmp.setY0(P0);
gmp.setGoal(Pg);

P_data = zeros(n_dof, n_data);
P_dot_data = zeros(n_dof, n_data);
P_ddot_data = zeros(n_dof, n_data);
for j=1:n_data
    x = xd_data(j);
	P_data(:,j) = gmp.getYd(x);
    P_dot_data(:,j) = gmp.getYdDot(x, x_dot);
    P_ddot_data(:,j) = gmp.getYdDDot(x, x_dot, x_ddot);
end

%% ====== Numerical diff on train data to get Vel/Accel ======
Pd_dot_data = zeros(size(Pd_data));
Pd_ddot_data = zeros(size(Pd_data));
dTime = diff(Timed);
for i=1:size(Pd_data,1)
    Pd_dot_data(i,:) = [diff(Pd_data(i,:))./dTime 0];
    Pd_ddot_data(i,:) = [diff(Pd_dot_data(i,:))./dTime 0];
end

%% ===========  Trajectories  ===========
figure;
k = [1 4 7];
for i=1:3
    subplot(3,3, k(1)); hold on;
    plot(Time, Pd_data(i,:), 'LineWidth',2, 'Color','magenta', 'LineStyle',':');
    plot(Time, P_data(i,:), 'LineWidth',2, 'Color','blue');
    axis tight;
    if (i==1), ylabel('pos [$m$]', 'interpreter','latex', 'fontsize',15); end
    
    subplot(3,3, k(2)); hold on;
    plot(Time, Pd_dot_data(i,:), 'LineWidth',2, 'Color','magenta', 'LineStyle',':');
    plot(Time, P_dot_data(i,:), 'LineWidth',2, 'Color','blue');
    axis tight;
    ylim([min(P_dot_data(i,:)) max(P_dot_data(i,:))]);
    if (i==1), ylabel('vel [$m/s$]', 'interpreter','latex', 'fontsize',15); end
    
    subplot(3,3, k(3)); hold on;
    plot(Time, Pd_ddot_data(i,:), 'LineWidth',2, 'Color','magenta', 'LineStyle',':');
    plot(Time, P_ddot_data(i,:), 'LineWidth',2, 'Color','blue');
    axis tight;
    ylim([min(P_ddot_data(i,:)) max(P_ddot_data(i,:))]);
    if (i==1), ylabel('accel [$m/s^2$]', 'interpreter','latex', 'fontsize',15); end
    
    k = k + 1;
end

gmp_.write(gmp, gmp_out_model_file);

% fid = FileIO(train_data_file, bitor(FileIO.out, FileIO.trunc));
% fid.write('Timed',Time);
% fid.write('Pd_data',P_data);
% fid.write('joint_pos_data',joint_pos_data);



