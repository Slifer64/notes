clc;
close all;
clear;

%% includes
import_io_lib();
import_math_lib();
import_plot_lib();
% import_gmp_lib();
addpath('./utils/');

data_path = './data/';
% data_path = '../config/';

cnt = num2str(1);

%% Load exec data
pick_frw = FileIO([data_path cnt '_exec_pick_frw.bin'], FileIO.in).readAll();
place_frw = FileIO([data_path cnt '_exec_place_frw.bin'], FileIO.in).readAll();

%% ========  trajectories  ==========
plot_pos_trajectories(pick_frw, place_frw, 1, 0)
% plot_orient_trajectories(pick_frw, place_frw, 1, 0);

%% ======== phase variable ==========
plot_phase_var(pick_frw, place_frw);


%% ======== plot 3D paths ========

target_vp_data = get_target_viapoints(place_frw);

y0 = pick_frw.P_data(1:3, 1);
Q0 = pick_frw.P_data(4:7, 1);

yg = pick_frw.P_data(1:3, end);
Qg = pick_frw.P_data(4:7, end);

label_font = 17;
ax_fontsize = 14;

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
plotOrientPath(ax, pick_frw.P_data(1:3,:), pick_frw.P_data(4:7,:), ...
    'LineWidth',2.5, 'Color',[0.6, 0.2, 0], 'LineStyle','-', 'FrameAxLen',0.1, 'addFrameDist',0.3, 'DisplayName','forward')
plot3(pick_frw.Pg_data(1,:), pick_frw.Pg_data(2,:), pick_frw.Pg_data(3,:), 'LineWidth',2, 'Color',[1 0.4 0.4]);
% place
plotOrientPath(ax, place_frw.P_data(1:3,:), place_frw.P_data(4:7,:), ...
    'LineWidth',2.5, 'Color',[0.96 0.69 0.13], 'LineStyle','-', 'FrameAxLen',0.1, 'addFrameDist',0.3, 'DisplayName','forward')
plot3(place_frw.Pg_data(1,:), place_frw.Pg_data(2,:), place_frw.Pg_data(3,:), 'LineWidth',2, 'Color',[1 0.4 0.4]);

for j=1:length(target_vp_data)
    opacity = (j/length(target_vp_data))^0.5;
    plotOrientPath(ax, target_vp_data{j}.pos, target_vp_data{j}.quat, ...
        'FrameXcolor',[1, 0, 0.5, opacity], 'FrameYcolor',[0.4, 1, 0.4, opacity], 'FrameZcolor',[0, 0.5, 1, opacity], ...
        'LineWidth',2.5, 'Color',[0.96 0.69 0.13], 'LineStyle','-', ...
        'FrameAxLen',0.1, 'addFrameDist',0.05, 'HandleVisibility','off')
end

% -----------------------------------------
legend({}, 'interpreter','latex', 'fontsize',17, 'Position',[0.8174 0.6641 0.1531 0.3144]);
view(-16.1, 51.2);
grid on;
% ax.XLim=x_lim; ax.YLim=y_lim; ax.ZLim=z_lim;
ax.FontSize = 14;
xlabel('x [$m$]', 'interpreter','latex', 'fontsize',18);
ylabel('y [$m$]', 'interpreter','latex', 'fontsize',18);
zlabel('z [$m$]', 'interpreter','latex', 'fontsize',18);
title('Cartesian path', 'interpreter','latex', 'fontsize',20);
hold(ax, 'off');


%% Velocity - Accel norm
plot_vel_accel_norms(pick_frw, place_frw);

%% =========================================
%% =========================================

function plot_phase_var(seg1, seg2)

if (seg2.t_data(1) ~= seg1.t_data(end))
    seg2.t_data = seg2.t_data - seg2.t_data(1) + seg1.t_data(end);
end

fig = figure;
ax_font = 13;
leg_font = 15;
label_font = 17;
ax_vec = [];

Time = [seg1.t_data seg2.t_data];
force_norm = vecnorm([seg1.Fext_data(1:3,:) seg2.Fext_data(1:3,:)],2,1);
max_force = max(force_norm);

ax = axes('Parent',fig);
hold(ax, 'on'); ax.Box='off';
grid(ax,'on');
plot(seg1.t_data, seg1.s_data(1,:), 'LineStyle','-', 'LineWidth',2, 'Color','blue', 'DisplayName','pick');
plot(seg2.t_data, seg2.s_data(1,:), 'LineStyle','-', 'LineWidth',2, 'Color','cyan', 'DisplayName','place');
ax.FontSize = ax_font;
legend({}, 'fontsize',15, 'interpreter','latex', 'Position',[0.0583 0.9349 0.4577 0.0512], 'Orientation','horizontal', 'Box','off');
ylabel("phase variable", 'fontsize',label_font, 'interpreter','latex');
xlabel("time [$s$]", 'fontsize',label_font, 'interpreter','latex');
axis(ax, 'tight');
ax.YLim = ax.YLim + 0.04*(ax.YLim(2)-ax.YLim(1))*[-1 1];

ax0 = axes('Parent',fig);
plot(Time, force_norm, 'LineWidth',2, 'LineStyle','-', 'Color',[0.96 0.69 0.13], 'DisplayName','$||f_{ext}||$');
axis(ax0, 'tight');
legend({}, 'fontsize',leg_font, 'interpreter','latex', 'Position',[0.7284 0.9349 0.1729 0.0457], 'Orientation','horizontal', 'Box','off');
set(ax0, 'YAxisLocation','right', 'XAxisLocation','top', 'XTick',[], 'fontsize',ax_font, 'color','none', 'Box','off');
ylabel("[N]", 'fontsize',label_font, 'interpreter','latex');
ax0.Position = ax.Position;
ax0.YLim = ax0.YLim + 0.04*(ax0.YLim(2)-ax0.YLim(1))*[-1 1];

end