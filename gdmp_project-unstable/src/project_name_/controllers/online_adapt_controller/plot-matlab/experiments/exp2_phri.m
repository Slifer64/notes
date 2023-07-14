clc;
close all;
clear;

%% includes
import_io_lib();
import_math_lib();
import_plot_lib();
addpath('./utils/');

data_path = './data/';

cnt = num2str(2);

%% Load exec data
seg1 = FileIO([data_path cnt '_exec_pick_frw.bin'], FileIO.in).readAll();
seg2 = FileIO([data_path cnt '_exec_place_frw.bin'], FileIO.in).readAll();


[Time0, P0_data, dP0_data, ddP0_data] = getModelTrajectory(seg1);


seg2.t_data = seg2.t_data - seg2.t_data(1) + seg1.t_data(end);

%% ======== plot x-position and force ========

fig = figure;
ax_font = 13;
leg_font = 15;
label_font = 17;
ax_vec = [];

Time = seg1.t_data;
force_data = seg1.Fext_data(1,:);
pos_data = seg1.P_data(1,:);
target_data = seg1.Pg_data(1,:);
pos0_data = P0_data(1,:);

ax = axes('Parent',fig);
hold(ax, 'on'); 
ax.Box='off';
grid(ax, 'on');
plot(Time, pos_data, 'LineWidth',2.5, 'LineStyle','-', 'Color',[0.6, 0.2, 0], 'DisplayName','modified DMP', 'Parent',ax);
plot(Time0, pos0_data, 'LineWidth',2.5, 'LineStyle','--', 'Color',[1, 0, 1], 'DisplayName','nominal DMP', 'Parent',ax);
plot(Time(1), pos_data(1), 'LineWidth', 3, 'LineStyle','none', 'Color',[0 0.9 0],'Marker','o', 'MarkerSize',10, 'DisplayName','$y_0$', 'Parent',ax, 'HandleVisibility','off');
plot(Time(end), target_data(end), 'LineWidth', 3, 'LineStyle','none', 'Color',[1 0 0],'Marker','x', 'MarkerSize',11, 'DisplayName','$g$', 'Parent',ax, 'HandleVisibility','off');
axis(ax, 'tight');
ax.FontSize = ax_font;
legend({}, 'fontsize',leg_font, 'interpreter','latex', 'Position',[0.0508 0.9235 0.6292 0.0657], 'Orientation','horizontal', 'Box','off');
ylabel("$x$-pos [$m$]", 'fontsize',label_font, 'interpreter','latex');
xlabel("time [$s$]", 'fontsize',label_font, 'interpreter','latex');
axis(ax, 'tight');
ax.YLim = ax.YLim + 0.04*(ax.YLim(2)-ax.YLim(1))*[-1 1];
ax.XLim = ax.XLim + [-0.06 0.08];

ax0 = axes('Parent',fig);
plot(Time, force_data, 'LineWidth',2, 'LineStyle','-', 'Color',[0.96 0.69 0.13], 'DisplayName','$f_{ext,x}$');
axis(ax0, 'tight');
legend({}, 'fontsize',leg_font+2, 'interpreter','latex', 'Position',[0.7533 0.9249 0.1643 0.0657], 'Orientation','horizontal', 'Box','off');
set(ax0, 'YAxisLocation','right', 'XAxisLocation','top', 'XTick',[], 'fontsize',ax_font, 'color','none', 'Box','off');
ylabel("[N]", 'fontsize',label_font, 'interpreter','latex');
ax0.Position = ax.Position;
ax0.YLim = ax0.YLim + 0.04*(ax0.YLim(2)-ax0.YLim(1))*[-1 1];
ax0.XLim = ax0.XLim + [-0.06 0.08];

%% ======== plot 3D paths ========

target_vp_data = get_target_viapoints(seg2);

y0 = seg1.P_data(1:3, 1);
Q0 = seg1.P_data(4:7, 1);

yg = seg1.P_data(1:3, end);
Qg = seg1.P_data(4:7, end);

label_font = 17;
ax_fontsize = 14;

fig = figure;
fig.Position(3:4) = [815 716];
ax = axes();
hold(ax, 'on')
% -----------------------------------------
plot3(y0(1), y0(2), y0(3), 'LineWidth', 3, 'LineStyle','none', 'Color',[0.1,0.8,0.1],'Marker','o', 'MarkerSize',12, 'DisplayName','$p_0$', 'HandleVisibility','off');
plotFrame(ax, y0, Q0, 'AxisLength',0.1, 'AxisWidth',3);
plot3(yg(1), yg(2), yg(3), 'LineWidth', 2, 'LineStyle','none', 'Color',[1 0.5 0.5],'Marker','x', 'MarkerSize',12, 'DisplayName','$g$', 'HandleVisibility','off');
plotFrame(ax, yg, Qg, 'AxisLength',0.1, 'AxisWidth',4);
% -----------------------------------------
% pick
plot3(seg1.P_data(1,:), seg1.P_data(2,:), seg1.P_data(3,:), 'LineWidth',2.5, 'Color',[0.6, 0.2, 0], 'LineStyle','-', 'DisplayName','modified pick');

plot3(P0_data(1,:), P0_data(2,:), P0_data(3,:), 'LineWidth',2.5, 'Color',[1, 0, 1], 'LineStyle','--', 'DisplayName','nominal pick');

% place
plot3(seg2.P_data(1,:), seg2.P_data(2,:), seg2.P_data(3,:), 'LineWidth',2.5, 'Color',[0.85 0.5 0.1], 'LineStyle','-', 'DisplayName','place');

plot3(seg2.Pg_data(1,:), seg2.Pg_data(2,:), seg2.Pg_data(3,:), 'LineWidth',2, 'LineStyle',':', 'Color',[1 0.4 0.4], 'HandleVisibility','off');

for j=1:length(target_vp_data)
    opacity = (j/length(target_vp_data))^1.2;
    plotOrientPath(ax, target_vp_data{j}.pos, target_vp_data{j}.quat, ...
        'FrameXcolor',[1, 0, 0.5, opacity], 'FrameYcolor',[0.4, 1, 0.4, opacity], 'FrameZcolor',[0, 0.5, 1, opacity], ...
        'LineWidth',2.5, 'Color',[0.96 0.69 0.13], 'LineStyle','-', ...
        'FrameAxLen',0.1, 'addFrameDist',0.05, 'HandleVisibility','off')
end

% plot obstacle
xb = [0.55 0.9];
yb = [-0.08 0.08];
zb = [0 0.35];

P = [mean(xb), mean(yb), mean(zb)] ; % center point 
L = [xb(2)-xb(1), yb(2)-yb(1), zb(2)-zb(1)] ; % cube dimensions 
plotcube(L, P-L/2, 0.35, [0.4 0.4 0.4], ax);

% plot carton
P = seg1.Pg_data(1:3,end)';
L = [0.2, 0.07, 0.07] ; % cube dimensions 
box_planes = plotcube(L, P-L/2, 0.35, [0.96 0.75 0.4], ax); %[0.96 0.75 0.4]
for i=1:length(box_planes)
    rotate(box_planes{i}, [0 0 1], 20);
    box_planes{i}.XData = box_planes{i}.XData + 0.12;
%     box_planes{i}.YData = box_planes{i}.YData - P(2);
%     box_planes{i}.ZData = box_planes{i}.ZData - P(3);
end
    
% -----------------------------------------
legend({}, 'interpreter','latex', 'fontsize',17, 'Position',[0.2480 0.7829 0.6346 0.0483], 'Orientation','Horizontal', 'Box','off');
view(106.23, 20.48);
grid on;
% ax.XLim=x_lim; ax.YLim=y_lim; ax.ZLim=z_lim;
ax.FontSize = 14;
xlabel('x [$m$]', 'interpreter','latex', 'fontsize',18);
ylabel('y [$m$]', 'interpreter','latex', 'fontsize',18);
zlabel('z [$m$]', 'interpreter','latex', 'fontsize',18);
% title('Cartesian path', 'interpreter','latex', 'fontsize',20);
hold(ax, 'off');

axis(ax, 'equal');

annotation(fig, 'textbox', 'String','Obstacle', 'Color',[1 0 0], 'FontSize',16, 'LineStyle','none', 'Position',[0.5878 0.4617 0.1508 0.0536]);

annotation(fig, 'textbox', 'String','Carton', 'Color',[0 0 0.8], 'FontSize',16, 'LineStyle','none', 'Position',[0.5768 0.2355 0.1214 0.0536]);
annotation(fig, 'arrow', 'Color',[0 0 0.8], 'LineWidth',2, 'LineStyle','-', 'Position',[0.6824 0.2769 0.0412 0.0659]);

annotation(fig, 'textbox', 'String','Target Box via-points', 'Color',[0 0 0.8], 'FontSize',16, 'LineStyle','none', 'Position',[0.2357 0.2388 0.2134 0.1061]);
annotation(fig, 'arrow', 'Color',[0.4 0.4 0.8], 'LineWidth',2, 'LineStyle','-', 'Position',[0.3523 0.3384 0.0796 0.0946]);
annotation(fig, 'textbox', 'String','initial', 'Color',[0.4 0.4 0.8], 'FontSize',16, 'LineStyle','none', 'Position',[0.4148 0.3673 0.1054 0.0614]);
annotation(fig, 'arrow', 'Color',[0 0 0.8], 'LineWidth',2, 'LineStyle','-', 'Position',[0.2849 0.3411 0.0587 0.1100]);
annotation(fig, 'textbox', 'String','final', 'Color',[0 0 0.8], 'FontSize',18, 'LineStyle','none', 'Position',[0.2173 0.3659 0.0895 0.0614]);
