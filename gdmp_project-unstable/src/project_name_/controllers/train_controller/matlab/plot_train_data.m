clc;
close all;
clear;

import_io_lib();
import_plot_lib();

filename = 'mpc_train_data2.bin';

dat = FileIO(['../config/training/' filename], FileIO.in).readAll();

Time = dat.Timed;
Pos = dat.Pd_data;
Quat = dat.Qd_data;

n_data = length(Time);

dTime = diff(Time);
dTime = [dTime dTime(end)];

Vel = zeros(3, n_data);
Accel = zeros(3, n_data);
for i=1:3
    Vel(i,:) = [diff(Pos(i,:)) 0] ./ dTime;
    Accel(i,:) = [diff(Vel(i,:)) 0] ./ dTime;
end

%% ===========  3D path  ===========
figure;
ax = plot_3Dpath_with_orientFrames(Pos(:,1:10:end), Quat(:,1:10:end), 'numberOfFrames',6, 'LineWidth',3, 'frameLineWidth',2, 'frameScale',0.3, 'animated',false);
axis tight;

%% ===========  Trajectories  ===========
figure;
k = [1 4 7];
for i=1:3
    subplot(3,3, k(1)); hold on;
    plot(Time, Pos(i,:), 'LineWidth',2, 'Color','blue');
    if (i==1), ylabel('pos [$m$]', 'interpreter','latex', 'fontsize',15); end
    
    subplot(3,3, k(2)); hold on;
    plot(Time, Vel(i,:), 'LineWidth',2, 'Color','blue');
    if (i==1), ylabel('vel [$m/s$]', 'interpreter','latex', 'fontsize',15); end
    
    subplot(3,3, k(3)); hold on;
    plot(Time, Accel(i,:), 'LineWidth',2, 'Color','blue');
    if (i==1), ylabel('accel [$m/s^2$]', 'interpreter','latex', 'fontsize',15); end
    
    k = k + 1;
end




