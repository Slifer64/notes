clc;
close all;
clear;

%% includes
addpath('utils/');
import_io_lib();
% import_gmp_lib();

data_path = './exp_data/';
% data_path = '../config/';


cnt = num2str(2);

%% Load exec data
pick_frw = FileIO([data_path cnt '_exec_pick_frw.bin'], FileIO.in).readAll();
pick_rev = FileIO([data_path cnt '_exec_pick_rev.bin'], FileIO.in).readAll();

place_frw = FileIO([data_path cnt '_exec_place_frw.bin'], FileIO.in).readAll();
place_rev = FileIO([data_path cnt '_exec_place_rev.bin'], FileIO.in).readAll();

y0 = pick_frw.P_data(1:3, 1);
Q0 = pick_frw.P_data(4:7, 1);

yg = pick_frw.P_data(1:3, end);
Qg = pick_frw.P_data(4:7, end);

%% ======= Plot results =========
label_font = 17;
ax_fontsize = 14;

vp_color = {[1, 0, 0.5, 0.7], [0.4, 1, 0.4, 0.7], [0, 0.5, 1, 0.7]};

%% Cartesian path

ind_v = pick_frw.sv_data >= 0;
sv_data = pick_frw.sv_data(ind_v);
Pv_data = pick_frw.Pv_data(:, ind_v);


% vp_offsets = [0.12, 0.08, 0.05, 0.03, 0.015];
% Place_vp = cell(length(vp_offsets), 1);
% for j=1:size(place_frw.Pg_data,2))
%     Place_vp
% end

%% --------- plot 3D paths ----------
fig = figure;
fig.Position(3:4) = [815 716];
ax = axes();
hold(ax, 'on')
% -----------------------------------------
plot3(y0(1), y0(2), y0(3), 'LineWidth', 4, 'LineStyle','none', 'Color',[0.47,0.67,0.19],'Marker','o', 'MarkerSize',12, 'DisplayName','$p_0$');
plotFrame(ax, y0, Q0, 'AxisLength',0.1, 'AxisWidth',2);
plot3(yg(1), yg(2), yg(3), 'LineWidth', 4, 'LineStyle','none', 'Color','red','Marker','x', 'MarkerSize',12, 'DisplayName','$g$');
plotFrame(ax, yg, Qg, 'AxisLength',0.1, 'AxisWidth',2);
% -----------------------------------------
% pick
plotOrientPath(ax, pick_frw.P_data(1:3,:), pick_frw.P_data(4:7,:), 'LineWidth',2.5, 'Color',[0.6, 0.2, 0], 'LineStyle','-', 'FrameAxLen',0.1, 'DisplayName','forward')
plot3(pick_rev.P_data(1,:), pick_rev.P_data(2,:), pick_rev.P_data(3,:), 'LineWidth',2.5, 'Color',[0.9, 0.6, 0.2], 'LineStyle','--', 'DisplayName','reverse');
if (~isempty(sv_data))
    plotOrientPath(ax, Pv_data(1:3,:), Pv_data(4:7,:), 'LineWidth',2, 'Color','blue', 'LineStyle','-', ...
        'FrameAxLen',0.1, 'addFrameDist',0.05, 'drawPath',false, ...
        'FrameXcolor',vp_color{1}, 'FrameYcolor',vp_color{2}, 'FrameZcolor',vp_color{3},...
        'FrameAxLen',0.15, 'FrameAxWidth',4);
end
% place
plotOrientPath(ax, place_frw.P_data(1:3,:), place_frw.P_data(4:7,:), 'LineWidth',2.5, 'Color',[0.6, 0.2, 0], 'LineStyle','-', 'FrameAxLen',0.1, 'DisplayName','forward')
plot3(place_rev.P_data(1,:), place_rev.P_data(2,:), place_rev.P_data(3,:), 'LineWidth',2.5, 'Color',[0.9, 0.6, 0.2], 'LineStyle','--', 'DisplayName','reverse');
% -----------------------------------------
legend({}, 'interpreter','latex', 'fontsize',17, 'Position',[0.8174 0.6641 0.1531 0.3144]);
view(130.6, 54);
grid on;
% ax.XLim=x_lim; ax.YLim=y_lim; ax.ZLim=z_lim;
ax.FontSize = 14;
xlabel('x [$m$]', 'interpreter','latex', 'fontsize',18);
ylabel('y [$m$]', 'interpreter','latex', 'fontsize',18);
zlabel('z [$m$]', 'interpreter','latex', 'fontsize',18);
title('Cartesian path', 'interpreter','latex', 'fontsize',20);
hold(ax, 'off');


%% Velocity - Accel norm

pick_frw = calcVelAccelNorm(pick_frw);
pick_rev = calcVelAccelNorm(pick_rev);

pick_rev.t_data = pick_rev.t_data + pick_frw.t_data(end);

figure;
subplot(2, 2, 1); hold on;
plot(pick_frw.t_data, pick_frw.v_norm, 'LineWidth',2, 'Color','blue');
plot(pick_rev.t_data, pick_rev.v_norm, 'LineWidth',2, 'LineStyle','-', 'Color','cyan');
ylabel('vel $m/s$', 'interpreter','latex', 'fontsize',14);
title('Translation', 'interpreter','latex', 'fontsize',16);
axis tight;
subplot(2, 2, 2); hold on;
plot(pick_frw.t_data, pick_frw.vRot_norm, 'LineWidth',2, 'Color','blue');
plot(pick_rev.t_data, pick_rev.vRot_norm, 'LineWidth',2, 'LineStyle','-', 'Color','cyan');
ylabel('rot-vel $rad/s$', 'interpreter','latex', 'fontsize',14);
title('Rotation', 'interpreter','latex', 'fontsize',16);
axis tight;
legend({'forward','reverse'}, 'interpreter','latex', 'fontsize',16);
subplot(2, 2, 3); hold on;
plot(pick_frw.t_data, pick_frw.vdot_norm, 'LineWidth',2, 'Color','blue');
plot(pick_rev.t_data, pick_rev.vdot_norm, 'LineWidth',2, 'LineStyle','-', 'Color','cyan');
ylabel('accel $m/s^2$', 'interpreter','latex', 'fontsize',14);
xlabel('time [$s$]', 'interpreter','latex', 'fontsize',14);
axis tight;
subplot(2, 2, 4); hold on;
plot(pick_frw.t_data, pick_frw.vRot_dot_norm, 'LineWidth',2, 'Color','blue');
plot(pick_rev.t_data, pick_rev.vRot_dot_norm, 'LineWidth',2, 'LineStyle','-', 'Color','cyan');
ylabel('rot-accel $rad/s^2$', 'interpreter','latex', 'fontsize',14);
xlabel('time [$s$]', 'interpreter','latex', 'fontsize',14);
axis tight;

return

%% trajectories

title_ = {'$X$-axis', '$Y$-axis', '$Z$-axis'};

fig = figure;
fig.Position(3:4) = [832 804];
% ======== position ==========
ax_ind = [1 2 3];
m = length(ax_ind);
sp_ind = 1:m:(2*m+1);
for i1=1:m
    
    i = ax_ind(i1);
    
    ax = subplot(3,m,sp_ind(1), 'Parent',fig);
    ax_vec = [ax];
    hold(ax, 'on')
    % plot position trajectory
    for k=1:length(data)
        if (~data{k}.plot2D), continue; end
        plot(data{k}.Time, data{k}.Pos(i,:), 'LineWidth',2.5, 'LineStyle',data{k}.linestyle, 'Color',data{k}.color, 'DisplayName', data{k}.legend, 'Parent',ax);
    end
    axis(ax, 'tight');
    % plot start and final positions
    plot(0, y0(i), 'LineWidth', 4, 'LineStyle','none', 'Color',[0.47,0.67,0.19],'Marker','o', 'MarkerSize',10, 'HandleVisibility','off', 'Parent',ax);
    plot(Tf, yg(i), 'LineWidth', 4, 'LineStyle','none', 'Color','red','Marker','x', 'MarkerSize',10, 'HandleVisibility','off', 'Parent',ax);
    % labels, title ...
    ylabel('pos [$m$]', 'interpreter','latex', 'fontsize',label_font, 'Parent',ax);
    title(title_{i}, 'interpreter','latex', 'fontsize',18, 'Parent',ax);
    %     title(title_{i}, 'interpreter','latex', 'fontsize',18);
    if (i1==1), legend(ax, {}, 'interpreter','latex', 'fontsize',17, 'Position',[0.3815 0.9458 0.2718 0.0416], 'Orientation', 'horizontal'); end
    ax.FontSize = ax_fontsize;
    hold(ax,'off')

    % ======== velocity ==========
    ax = subplot(3,m,sp_ind(2), 'Parent',fig);
    ax_vec = [ax_vec ax];
    hold(ax, 'on')
    plot(Tf, 0, 'LineWidth', 4, 'LineStyle','none', 'Color','red','Marker','x', 'MarkerSize',10, 'Parent',ax);
    for k=1:length(data)
        if (~data{k}.plot2D), continue; end
        plot(data{k}.Time, data{k}.Vel(i,:), 'LineWidth',2.5, 'LineStyle',data{k}.linestyle, 'Color',data{k}.color, 'Parent',ax);
    end
    axis(ax, 'tight');
    ylabel('vel [$m/s$]', 'interpreter','latex', 'fontsize',label_font, 'Parent',ax);
    ax.FontSize = ax_fontsize;
    hold(ax,'off')

    % ======== acceleration ==========
    ax = subplot(3,m,sp_ind(3), 'Parent',fig);
    ax_vec = [ax_vec ax];
    hold(ax, 'on')
    plot(Tf, 0, 'LineWidth', 4, 'LineStyle','none', 'Color','red','Marker','x', 'MarkerSize',10, 'Parent',ax);
    for k=1:length(data)
        if (~data{k}.plot2D), continue; end
        plot(data{k}.Time, data{k}.Accel(i,:), 'LineWidth',2.5, 'LineStyle',data{k}.linestyle, 'Color',data{k}.color, 'Parent',ax);
    end
    axis(ax, 'tight');
    ylabel('accel [$m/s^2$]', 'interpreter','latex', 'fontsize',label_font, 'Parent',ax);
    ax.FontSize = ax_fontsize;
    hold(ax, 'off');
    
    linkaxes(ax_vec,'x');
    
    sp_ind = sp_ind + 1;

end



function data = calcVelAccelNorm(data)

    data.v_norm = vecnorm(data.Vd_data(1:3, :), 2, 1);
    data.vdot_norm = vecnorm(data.Vd_dot_data(1:3, :), 2, 1);
    data.vRot_norm = vecnorm(data.Vd_data(4:6, :), 2, 1);
    data.vRot_dot_norm = vecnorm(data.Vd_dot_data(4:6, :), 2, 1);

end

